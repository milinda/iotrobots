package cgl.iotrobots.collavoid.planners;

import cgl.iotrobots.collavoid.commons.planners.*;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import geometry_msgs.*;
import nav_msgs.Odometry;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.*;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import sensor_msgs.PointCloud2;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;
import com.rabbitmq.client.*;

import javax.vecmath.Point3d;
import java.util.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Logger;

public class Agent {

    //config for simulation and some rules
    public boolean useTruancation;
    public double truncTime;
    public double simPeriod;

    //whether it is a controlled robot
    public boolean controlled;

    //setting for different CollAvoid strategies
    //orca or vo, need to figure out its usage
    public boolean orca;
    //footprint approximation, use circle approx or mink sum
    public boolean convex;

    //vo setting
    //clearPath or sampling based collAvoid strategy,use clearpath
    //boolean clearPath;

    //0:HRVO,1:RVO,2:VO
    public int voType;

    //robot information
    public Position position; //contains the heading information
    public Vector2 velocity;
    public Vector2 newVelocity;
    private double radius;
    public List<Vector2> footPrint_rotated;

    //allowed error for non-holonomic robot
    public double cur_allowed_error_;

    //orca
    public double max_speed_x_; //in nonholomic robot it has only one liner velocity
    public List<Line> orcaLines, addOrcaLines;

    //VO
    public List<VO> voAgents;
    public List<VelocitySample> samples;

    public List<Agent> AgentNeighbors;

    //config
    public double publishPositionsPeriod;
    public long publishMePeriod;//position share,in milli second

    public double thresholdLastSeen;
    public int numSamples;

    //helpers??
    public long lastTimePositionsPublished;//for visualization in rviz
    public long lastTimeMePublished; //for position share

    //NH stuff
    public double minErrorHolo;
    public double maxErrorHolo;
    public boolean holo_robot_;
    public double time_to_holo_;

    //ORCA stuff
    public double max_vel_with_obstacles_;
    public Vector2 holo_velocity_;

    //Obstacles
    public List<Obstacle> obstacles_from_laser_;

    public double min_dist_obst_;

    //obstacles
    public boolean use_obstacles_;
    public List<Vector2> obstacle_points_;

    //Agent description
    private String Name;
    public String base_frame_;
    public String global_frame_;
    public double wheel_base_;
    public List<Vector2> footprint_original;// 2D footprint in robot frame
    public double acc_lim_x_, acc_lim_y_, acc_lim_th_;
    public double min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, max_vel_th_, min_vel_th_, min_vel_th_inplace_;
    public double footprint_radius_;

    //set automatically
    public boolean initialized_;
    public boolean has_polygon_footprint_;

    public List<LinePair> footprint_original_lines;

    private long last_seen_;
    private Odometry_ base_odom_;

    //LOC uncertatiny
    public double eps_;
    public double cur_loc_unc_radius_;

    //COLLVOID
    public List<Vector2> footprint_minkowski;
    public List<PoseWeighted> pose_array_weighted_;

    //lock
    public Lock me_lock_, obstacle_lock_, neighbors_lock_, convex_lock_;

    //subscribers and publishers
    public Publisher lines_pub_, neighbors_pub_, polygon_pub_, vo_pub_, me_pub_, samples_pub_, speed_pub_, position_share_pub_, obstacles_pub_;
    public Subscriber amcl_posearray_sub_, position_share_sub_, odom_sub_, laser_scan_sub_, laser_notifier;

    //utils
    private static Logger logger;

    private RMQMsgManager rmqMsgManager;

    //-----------------------method begin-------------------------------//
    public Agent(String name,
                 Address[] addresses,
                 String url) {
        Name = name;
        me_lock_ = new ReentrantLock();
        obstacle_lock_ = new ReentrantLock();
        neighbors_lock_ = new ReentrantLock();
        convex_lock_ = new ReentrantLock();

        initialized_ = false;
        cur_allowed_error_ = 0;
        cur_loc_unc_radius_ = 0;
        min_dist_obst_ = Double.MAX_VALUE;

        holo_velocity_ = new Vector2();

        base_odom_ = new Odometry_();

        footprint_original = new ArrayList<Vector2>();
        footprint_minkowski = new ArrayList<Vector2>();
        footprint_original_lines = new ArrayList<LinePair>();
        obstacle_points_ = new ArrayList<Vector2>();
        pose_array_weighted_ = new ArrayList<PoseWeighted>();
        AgentNeighbors = new ArrayList<Agent>();
        obstacles_from_laser_ = new ArrayList<Obstacle>();
        addOrcaLines = new ArrayList<Line>();
        voAgents = new ArrayList<VO>();
        samples = new ArrayList<VelocitySample>();
        rmqMsgManager = new RMQMsgManager(name, addresses, url);
        initParameters();
    }

    // for neighbor recording
    public Agent(String name) {
        Name = name;
        footprint_original = new ArrayList<Vector2>();
        base_odom_ = new Odometry_();
        holo_velocity_ = new Vector2();
        position = new Position();
        footprint_minkowski = new ArrayList<Vector2>();
    }

    void initParameters() {
        double controller_frequency = -1;

        //load parameters locally
        use_obstacles_ = Parameters.USE_OBSTACLES;
        controlled = Parameters.CONTROLLED;

        base_frame_ = Name.replace("/", "") + Parameters.BASE_FRAME_SUFFIX;//get rid of the / character
        global_frame_ = Parameters.GLOBAL_FRAME;

        //acceleration limits load from params_turtle.yaml
        acc_lim_x_ = Parameters.ACC_LIM_X;
        acc_lim_y_ = Parameters.ACC_LIM_Y;
        acc_lim_th_ = Parameters.ACC_LIM_TH;

        //holo_robot
        holo_robot_ = Parameters.HOLO_ROBOT;
        if (!holo_robot_)
            wheel_base_ = Parameters.WHEEL_BASE;
        else
            wheel_base_ = 0.0;

        //min max speeds
        max_vel_with_obstacles_ = Parameters.MAX_VEL_WITH_OBSTACLES;
        max_vel_x_ = Parameters.MAX_VEL_X;
        min_vel_x_ = Parameters.MIN_VEL_X;
        max_vel_y_ = Parameters.MAX_VEL_Y;
        min_vel_y_ = Parameters.MIN_VEL_Y;
        max_vel_th_ = Parameters.MAX_VEL_TH;
        min_vel_th_ = Parameters.MIN_VEL_TH;
        min_vel_th_inplace_ = Parameters.MIN_VEL_TH_INPLACE;

        //set radius
        footprint_radius_ = Parameters.FOOTPRINT_RADIUS;

        //sim period
        controller_frequency = Parameters.CONTROLLER_FREQUENCY;

        //me.time_horizon_obst_ = getParamDef(private_nh,"time_horizon_obst",10.0); currently not used in agent
        //non holo robot parameters
        time_to_holo_ = Parameters.TIME_TO_HOLO;
        minErrorHolo = Parameters.MIN_ERROR_HOLO;
        maxErrorHolo = Parameters.MAX_ERROR_HOLO;
        //delete_observations_ = params.getBoolean("delete_observations", true); currently not used in agent
        //threshold_last_seen_ = params.getDouble("threshold_last_seen",1.0); currently not used in agent

        //parameters for convex footprint which considers localization uncertainty
        eps_ = Parameters.EPS;
        orca = Parameters.ORCA;
        convex = Parameters.CONVEX;
        //params.getBoolean( "clearpath", &clearpath); not used as we only use clear path method
        useTruancation = Parameters.USE_TRUNCATION;

        //num_samples = getParamDef(private_nh, "num_samples", 400); not used
        voType = Parameters.TYPE_VO; //HRVO

        truncTime = Parameters.TRUNC_TIME;
        //left_pref_ = getParamDef(private_nh,"left_pref",0.1); not used as it is for orca method

        //visualization publish frequency
        publishPositionsPeriod = 1.0 / Parameters.PUBLISH_POSITIONS_FREQUENCY;
        //position share publish frequency
        publishMePeriod = (long) (1.0 / Parameters.PUBLISH_ME_FREQUENCY * 1000);

        radius = footprint_radius_ + cur_loc_unc_radius_;

        if (controller_frequency <= 0) {
            logger.warning("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
            simPeriod = 0.05;
        } else {
            simPeriod = 1.0 / controller_frequency;
        }
        logger.info("Sim period is set to " + String.format("%1$.2f", simPeriod));


        //currently use circular footprint
        footprint_original.clear();
        double angle = 0;
        double step = 2 * Math.PI / 72;
        while (angle < 2 * Math.PI) {
            Vector2 pt = new Vector2();
            pt.setX((float) (footprint_radius_ * Math.cos(angle)));
            pt.setY((float) (footprint_radius_ * Math.sin(angle)));
            footprint_original.add(pt);
            angle += step;
        }
        setFootprint(footprint_original);
        initialized_ = true;
    }


    public void initPubSub(final ConnectedNode newnode) {
        //---------------test area
        ctlCmdPub = node.newPublisher("/ctl_cmd", std_msgs.String._TYPE);

        //Publishers, most for visualization purpose
        neighbors_pub_ = node.newPublisher(Name + "/v_neighbors", MarkerArray._TYPE);
        me_pub_ = node.newPublisher(Name + "/v_ROSAgent", MarkerArray._TYPE);
        //convex hull foot print taking localization uncertainty into account
        polygon_pub_ = node.newPublisher(Name + "/convex_hull", PolygonStamped._TYPE);
        vo_pub_ = node.newPublisher(Name + "/v_vo", Marker._TYPE);
        lines_pub_ = node.newPublisher(Name + "/v_orca_lines", Marker._TYPE);
        obstacles_pub_ = node.newPublisher(Name + "/v_obstacles", Marker._TYPE);
        samples_pub_ = node.newPublisher(Name + "/v_samples", MarkerArray._TYPE);
        speed_pub_ = node.newPublisher(Name + "/v_speed", Marker._TYPE);

        position_share_pub_ = node.newPublisher("/position_share", pose_twist_covariance_msgs._TYPE);

        //Subscribers
        position_share_sub_ = node.newSubscriber("/position_share", pose_twist_covariance_msgs._TYPE);
        position_share_sub_.addMessageListener(new MessageListener<pose_twist_covariance_msgs>() {
            @Override
            public void onNewMessage(pose_twist_covariance_msgs msg) {
                positionShareCallback(msg);
            }
        }, 10);

        /*TODO: Need particle cloud and the weights of the particles to calculate localization uncertainty,
          TODO: but currently just particle cloud.*/
        amcl_posearray_sub_ = node.newSubscriber(Name + "/particlecloud", PoseArray._TYPE);
        amcl_posearray_sub_.addMessageListener(new MessageListener<PoseArray>() {
            @Override
            public void onNewMessage(PoseArray msg) {
                //amclPoseArrayWeightedCallback(msg);
                PoseArrayTestCallback(msg);
            }
        });

        odom_sub_ = node.newSubscriber(Name + "/odometry", Odometry._TYPE);
        odom_sub_.addMessageListener(new MessageListener<Odometry>() {
            @Override
            public void onNewMessage(Odometry msg) {
                odomCallback(msg);
            }
        });

//        //laser_scan_sub_.subscribe (nh, "base_scan", 1);
//        //laser_notifier.reset(new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, global_frame_, 10));
        laser_scan_sub_ = node.newSubscriber(Name + "/scan/point_cloud2", PointCloud2._TYPE);
        laser_scan_sub_.addMessageListener(new MessageListener<PointCloud2>() {
            @Override
            public void onNewMessage(PointCloud2 msg) {
                Duration dur = Duration.fromMillis(100);
                baseScanCallback(msg, dur);

            }
        }, 10);

//        laser_notifier->registerCallback(boost::bind(&ROSAgent::baseScanCallback, this, _1));
//        laser_notifier->setTolerance(ros::Duration(0.1));

        logger.info("************************" + Name + " is initialized.");

    }


    // original test case
    void PoseArrayTestCallback(PoseArray msg) {
        // in robot base frame do not need transform
        double x, y;
        List<Vector2> localization_footprint = new ArrayList<Vector2>();
        List<Vector2> own_footprint = new ArrayList<Vector2>();
        for (int i = 0; i < msg.getPoses().size(); i++) {
            x = msg.getPoses().get(i).getPosition().getX();
            y = msg.getPoses().get(i).getPosition().getY();
            Vector2 p = new Vector2(x, y);
            if (p.getLength() > 0.1)
                continue;
            localization_footprint.add(p);
        }

        //for test replaced the algorithm computeNewMinkowskiFootprint
        for (int i = 0; i < footprint_original_msg_.getPolygon().getPoints().size(); i++) {
            Point32 p = footprint_original_msg_.getPolygon().getPoints().get(i);
            own_footprint.add(new Vector2(p.getX(), p.getY()));
            //      ROS_WARN("footprint point p = (%f, %f) ", footprint_[i].x, footprint_[i].y);
        }
        footprint_minkowski = CP.minkowskiSumConvexHull(localization_footprint, own_footprint);
        //publish footprint
        PolygonStamped msg_pub = createFootprintMsgFromVector2(footprint_minkowski);
        polygon_pub_.publish(msg_pub);

    }

    void amclPoseArrayWeightedCallback(final PoseArray msg) {
        convex_lock_.lock();
        try {
            pose_array_weighted_.clear();
            PoseStamped in = messageFactory.newFromType(PoseStamped._TYPE);
            in.setHeader(msg.getHeader());
            for (int i = 0; i < msg.getPoses().size(); i++) {
                in.setPose(msg.getPoses().get(i));
                //TODO: must transform!!!! All footprint are in base frame.
/*                try {
                    tf_ -> waitForTransform(global_frame_, base_frame_, in.header.stamp, ros::Duration (0.2));
                    tf_ -> transformPose(base_frame_, in, result);*/
                //TODO: need weight
                PoseWeighted result = new PoseWeighted(0.0, in);
                pose_array_weighted_.add(result);
 /*               } catch (tf::TransformException ex){
                    ROS_ERROR("%s", ex.what());
                    ROS_ERROR("point transform failed");
                }      */
            }

            if (!convex) {
                //just add the localization uncertainty to the radius
                computeNewLocUncertainty();
            } else {
                //use convex shape
                computeNewMinkowskiFootprint();
            }
        } finally {
            convex_lock_.unlock();
        }

    }

    // radius considering localization uncertainty
    void computeNewLocUncertainty() {
        List<ConvexHullPoint> points = new ArrayList<ConvexHullPoint>();
        for (int i = 0; i < pose_array_weighted_.size(); i++) {
            ConvexHullPoint p = new ConvexHullPoint();
            double x = pose_array_weighted_.get(i).getPoseStamped().getPose().getPosition().getX();
            double y = pose_array_weighted_.get(i).getPoseStamped().getPose().getPosition().getY();
            p.setPoint(new Vector2(x, y));
            p.setWeight(pose_array_weighted_.get(i).getW());
            p.setIndex(i);
            p.setOrig_index(i);
            points.add(p);
        }
        Collections.sort(points, new ConvexHullPointsPositionComparator());
        double sum = 0.0;
        int j = 0;
        while (sum <= 1.0 - eps_ && j < points.size()) {
            //TODO:need weight or the loop will not stop
            sum += points.get(j).getWeight();
            j++;
        }
        //maximum two times of the footpint radius
        cur_loc_unc_radius_ = Math.min(footprint_radius_ * 2.0, Vector2.abs(points.get(j - 1).getX(), points.get(j - 1).getY()));
        //ROS_ERROR("Loc Uncertainty = %f", cur_loc_unc_radius_);
        radius = footprint_radius_ + cur_loc_unc_radius_;
    }

    // MinkowskiFootprint considering localization uncertainty
    void computeNewMinkowskiFootprint() {
        ConvexHullPoint[] convex_hull = new ConvexHullPoint[0];
        ConvexHullPoint[] points = new ConvexHullPoint[pose_array_weighted_.size()];

        for (int i = 0; i < pose_array_weighted_.size(); i++) {
            ConvexHullPoint p = new ConvexHullPoint();
            double x = pose_array_weighted_.get(i).getPoseStamped().getPose().getPosition().getX();
            double y = pose_array_weighted_.get(i).getPoseStamped().getPose().getPosition().getY();
            p.setPoint(x, y);
            p.setWeight(pose_array_weighted_.get(i).getW());
            p.setIndex(i);
            p.setOrig_index(i);
            points[i] = p;
        }
        Arrays.sort(points, new VectorsLexigraphicComparator());
        for (int i = 0; i < points.length; i++) {
            points[i].setIndex(i);
        }

        double total_sum = 0;

        int[] skip_list;
        while (points.length >= 3) {
            double sum = 0;
            convex_hull = CP.convexHull(points, true);

            skip_list = new int[convex_hull.length];
            for (int j = 0; j < convex_hull.length - 1; j++) {
                skip_list[j] = convex_hull[j].getIndex();
                sum += convex_hull[j].getWeight();
            }
            total_sum += sum;

            //ROS_WARN("SUM = %f (%f), num Particles = %d, eps = %f", sum, total_sum, (int) points.size(), eps_);

            if (total_sum >= eps_) {
                break;
            }

            Arrays.sort(skip_list);
            ConvexHullPoint[] pointsTmp = new ConvexHullPoint[points.length - skip_list.length];
            int l = 0, k = 0;
            for (int j = 0; j < points.length; j++) {
                if (j != skip_list[l++]) {
                    ConvexHullPoint pt = new ConvexHullPoint(points[j].getX(), points[j].getY());
                    pt.setIndex(k);
                    pointsTmp[k++] = pt;
                }

            }
            points = pointsTmp;
        }

        List<Vector2> localization_footprint = new ArrayList<Vector2>();
        List<Vector2> own_footprint = new ArrayList<Vector2>();
        for (int i = 0; i < convex_hull.length; i++) {
            localization_footprint.add(new Vector2(convex_hull[i].getX(), convex_hull[i].getY()));
        }

        for (int i = 0; i < footprint_original_msg_.getPolygon().getPoints().size(); i++) {
            Point32 p = footprint_original_msg_.getPolygon().getPoints().get(i);
            own_footprint.add(new Vector2(p.getX(), p.getY()));
            //      ROS_WARN("footprint point p = (%f, %f) ", footprint_[i].x, footprint_[i].y);
        }
        footprint_minkowski = CP.minkowskiSumConvexHull(localization_footprint, own_footprint);
        //publish footprint
        PolygonStamped msg_pub = createFootprintMsgFromVector2(footprint_minkowski);
        // for visualization
        polygon_pub_.publish(msg_pub);

    }


    PolygonStamped createFootprintMsgFromVector2(final List<Vector2> footprint) {
        PolygonStamped result = messageFactory.newFromType(PolygonStamped._TYPE);
        result.getHeader().setFrameId(base_frame_);
        result.getHeader().setStamp(node.getCurrentTime());

        for (int i = 0; i < footprint.size(); i++) {
            Point32 p = messageFactory.newFromType(Point32._TYPE);
            p.setX((float) footprint.get(i).getX());
            p.setY((float) footprint.get(i).getY());
            result.getPolygon().getPoints().add(p);
        }
        return result;
    }


    public void positionShareCallback(final pose_twist_covariance_msgs msg) {
        //System.out.println("pshare sequence"+msg.getHeader().getSeq());

        neighbors_lock_.lock();
        try {

            String cur_id = msg.getRobotId();
            if (!cur_id.equals(Name)) {  //if it is not me do something
                int i;
                for (i = 0; i < AgentNeighbors.size(); i++) {

                    //ROSAgent ROSAgent = boost::dynamic_pointer_cast < ROSAgent > (agent_neighbors_[i]);

                    if (AgentNeighbors.get(i).Name.equals(cur_id)) {
                        //I found the robot
                        break;
                    }
                }
                if (i >= AgentNeighbors.size()) { //Robot is new, so it will be added to the list
                    ROSAgent new_robot = new ROSAgent(cur_id);
                    new_robot.holo_robot_ = msg.getHoloRobot();
                    AgentNeighbors.add(new_robot);
                    logger.info(Name + " added a new neighbor with AgentName " + cur_id + " and radius " + msg.getRadius());
                }

                ROSAgent lstagent = AgentNeighbors.get(i);
                lstagent.base_odom_.setPose(msg.getPose());
                lstagent.base_odom_.setTwist(msg.getTwist());
                lstagent.holo_velocity_ = new Vector2(msg.getHolonomicVelocity().getX(), msg.getHolonomicVelocity().getY());
                lstagent.radius = msg.getRadius();
                lstagent.controlled = msg.getControlled();
                lstagent.footprint_original_msg_ = msg.getFootprint();
                lstagent.setMinkowskiFootprintVector2(msg.getFootprint());
                lstagent.last_seen_ = msg.getHeader().getStamp();

            }

            if ((node.getCurrentTime().toSeconds() - lastTimePositionsPublished.toSeconds()) > publishPositionsPeriod) {
                lastTimePositionsPublished = node.getCurrentTime();
                //msgPublisher.publishNeighborPositions(AgentNeighbors, global_frame_, base_frame_, neighbors_pub_);// for visualization
                //in case odometry call back is not called at the beginning
                if (last_seen_ == null) {
                    last_seen_ = node.getCurrentTime();
                }
                //msgPublisher.publishMePosition(this, global_frame_, base_frame_, me_pub_);//for visualize in rviz
            }
        } finally {
            neighbors_lock_.unlock();
        }
    }

    public void setMinkowskiFootprintVector2(List<Vector2> footprint_ori) {
        footprint_minkowski.clear();
        for (Vector2 pt : footprint_ori) {
            footprint_minkowski.add(pt);
        }
    }

    void odomCallback(final Odometry msg) {
        // In the original program, odometry is published in robot frame, but it need velocities in
        // robot frame, and pose in global frame. So it directly cast the twist to base odom but
        // transformed the pose into global frame.
        // In our context both velocity and position are in odometry frame so only need to transform
        // velocity to robot frame. Currently map frame and odometry frame are the same.

        me_lock_.lock();
        try {
            Twist twist = node.getTopicMessageFactory().newFromType(Twist._TYPE);
            // here actually can just get the length of linear velocity instead
//            if (!tf_.transformTwist(base_frame_, global_frame_, msg.getTwist().getTwist(), twist)) {
//                node.getLog().error("Can not transform twist to base frame: " + msg.getHeader().getFrameId() + "->" + base_frame_);
//                return;
//            }
            twist.getLinear().setX(Vector2.abs(new Vector2(msg.getTwist().getTwist().getLinear().getX(), msg.getTwist().getTwist().getLinear().getY())));
            twist.getAngular().setZ(msg.getTwist().getTwist().getAngular().getZ());

            base_odom_.getTwist().setTwist(twist);// base frame
            base_odom_.getPose().setPose(msg.getPose().getPose());// global frame
            base_odom_.getHeader().setStamp(msg.getHeader().getStamp());
            base_odom_.getHeader().setSeq(msg.getHeader().getSeq());
            base_odom_.getHeader().setFrameId(base_frame_);

            last_seen_ = msg.getHeader().getStamp();


            if ((node.getCurrentTime().toSeconds() - lastTimeMePublished.toSeconds()) > publishMePeriod) {
                lastTimeMePublished = node.getCurrentTime();
                publishMePoseTwist();
            }
        } finally {
            me_lock_.unlock();
        }
    }

    void publishMePoseTwist() {
        pose_twist_covariance_msgs me_msg = messageFactory.newFromType(pose_twist_covariance_msgs._TYPE);
        me_msg.getHeader().setStamp(node.getCurrentTime());
        me_msg.getHeader().setFrameId(base_frame_);///velocity frame
        me_msg.getHeader().setSeq(base_odom_.getHeader().getSeq());

        me_msg.getPose().setPose(base_odom_.getPose().getPose());
        me_msg.getTwist().setTwist(base_odom_.getTwist().getTwist());

        me_msg.setControlled(controlled);
        me_msg.getHolonomicVelocity().setX(holo_velocity_.getX());
        me_msg.getHolonomicVelocity().setY(holo_velocity_.getY());

        me_msg.setHoloRobot(holo_robot_);
        me_msg.setRadius((float) (footprint_radius_ + cur_loc_unc_radius_));
        me_msg.setRobotId(Name);

        me_msg.setFootprint(createFootprintMsgFromVector2(footprint_minkowski));
        position_share_pub_.publish(me_msg);
    }

    // scans are published in global frame
    public void baseScanCallback(final PointCloud2 msg, Duration dur_m) {
//        if (Name.equals("robot0"))
//        System.out.println("Scan sequence    "+msg.getHeader().getSeq()%10);
        List<Point3d> point3ds = new ArrayList<Point3d>();

        if (msg.getWidth() * msg.getHeight() == 0) {
            obstacle_lock_.lock();
            try {
                obstacles_from_laser_.clear();
            } finally {
                obstacle_lock_.unlock();
            }
            return;
        }
        //System.out.println("received scan at x: "+base_odom_.getPose().getPose().getPosition().getX());

        ChannelBuffer data = msg.getData().copy();
        while (data.readableBytes() > 0) {
            double[] pt = new double[msg.getFields().size()];
            for (int k = 0; k < msg.getFields().size(); k++) {
                pt[k] = data.readFloat();
            }
            point3ds.add(new Point3d(pt));
        }
        //no need to transform rviz will transform automatically
//            if (!tf_.transformPoint3ds(global_frame_, base_frame_, point3ds, msg.getHeader().getStamp().totalNsecs(), dur_m)) {
//                node.getLog().error("Can not transform cloud points: " + base_frame_ + "->" + global_frame_);
//                return;
//            }

        obstacle_lock_.lock();
        try {
            obstacles_from_laser_.clear();
            double threshold_convex = 0.03;
            double threshold_concave = -0.03;
            //    ROS_ERROR("%d", (int)cloud.points.size());
            Vector2 start;
            for (int i = 0; i < point3ds.size(); i++) {
                start = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
                while (pointInNeighbor(start) && i < point3ds.size() - 1) {
                    i++;
                    start = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
                }
                if (i == point3ds.size()) {
                    //it is a agent
                    return;
                }

                boolean found = false;
                Vector2 prev = new Vector2(start);
                double first_ang = 0;
                double prev_ang = 0;
                Vector2 next;
                while (!found) {
                    i++;
                    if (i == point3ds.size()) {
                        break;
                    }
                    next = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
                    while (pointInNeighbor(next) && i < point3ds.size() - 1) {
                        i++;
                        next = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
                    }

                    if (Vector2.abs(Vector2.minus(next, prev)) > 2 * footprint_radius_) {
                        found = true;
                        break;
                    }
                    Vector2 dif = Vector2.minus(next, start);
                    double ang = Math.atan2(dif.getY(), dif.getX());
                    if (!prev.equals(start)) {
                        if (ang - first_ang < threshold_concave) {
                            found = true;
                            i -= 2;
                            break;
                        }
                        if (ang - prev_ang < threshold_concave) {
                            found = true;
                            i -= 2;
                            break;
                        }
                        if (ang - prev_ang > threshold_convex) { //going towards me
                            found = true;
                            i -= 2;
                            break;
                        }
                        if (ang - first_ang > threshold_convex) { //going towards me
                            found = true;
                            i -= 2;
                            break;
                        }

                    } else {
                        first_ang = ang;
                    }
                    prev = new Vector2(next);
                    prev_ang = ang;
                }
                Obstacle obst = new Obstacle(start, prev);
                obst.setTime(msg.getHeader().getStamp());

                obstacles_from_laser_.add(obst);
            }
        } finally {
            obstacle_lock_.unlock();
        }

        //tell its original frame
        //msgPublisher.publishObstacleLines(obstacles_from_laser_, global_frame_, base_frame_, obstacles_pub_);
    }

    boolean pointInNeighbor(Vector2 point) {
        double dist;
        for (int i = 0; i < AgentNeighbors.size(); i++) {
            dist = Vector2.abs(Vector2.minus(point, AgentNeighbors.get(i).position.getPos()));
            if (dist <= AgentNeighbors.get(i).radius) {
                return true;
            }
        }
        return false;
    }


    //called by local planner
    /* first refresh agent status according to the last velocity computed and then compute new
    * velocity */

    // cmd_vel is in robot frame that means it should have no Y velocity, just X and Z angular velocity
    public void computeNewVelocity(Vector2 pref_velocity, Twist_ cmd_vel) {
        me_lock_.lock();
        try {
            //Forward project agents,update me status like position and so on
            upDateAgentState(this);
            updateAllNeighbors();

            newVelocity = new Vector2(0.0, 0.0);

            addOrcaLines.clear();
            voAgents.clear();

            //get closest ROSAgent/obstacle
            double min_dist_neigh = Double.MAX_VALUE;
            if (AgentNeighbors.size() > 0)
                //neighbors have already been sorted according to their dist to me
                min_dist_neigh = Vector2.abs(Vector2.minus(AgentNeighbors.get(0).position.getPos(), position.getPos()));

//            System.out.println("min neighbor: "+min dist neigh);
//            System.out.println("obst_min: "+min dist obst_);
            double min_dist = Math.min(min_dist_neigh, min_dist_obst_);

            //incorporate NH constraints
            max_speed_x_ = max_vel_x_;//???????????????????????????

            if (!holo_robot_) {
                addNHConstraints(min_dist, pref_velocity);
            }
            //add acceleration constraints

            NHORCA.addAccelerationConstraintsXY(max_vel_x_, acc_lim_x_, max_vel_y_, acc_lim_y_, velocity, position.getHeading(), simPeriod, holo_robot_, addOrcaLines);

            computeObstacles();

            // currently only compute the clear path velocity which has the best performance as described in the thesis.

//            if (orca_) {
//                computeOrcaVelocity(pref_velocity);
//            } else {
            samples.clear();
//                if (clearpath_) {
            computeClearpathVelocity(pref_velocity);
//                } else {
//                    computeSampledVelocity(pref_velocity);
//                }
//            }


            double speed_ang = Math.atan2(newVelocity.getY(), newVelocity.getX());
            double dif_ang = Angles.shortest_angular_distance(position.getHeading(), speed_ang);

            Vector3 linear = messageFactory.newFromType(Vector3._TYPE);
            Vector3 angular = messageFactory.newFromType(Vector3._TYPE);
            if (!holo_robot_) {
                double vel = Vector2.abs(newVelocity);
                double vstar;

                if (Math.abs(dif_ang) > EPSILON.EPSILON)
                    vstar = NHORCA.calcVstar(vel, dif_ang);//get nonholomonic velocity
                else
                    vstar = max_vel_x_;

                linear.setX(Math.min(vstar, vMaxAng()));
                linear.setY(0.0);
                cmd_vel.setLinear(linear);

                //ROS_ERROR("dif_ang %f", dif_ang);
                if (Math.abs(dif_ang) > 3.0 * Math.PI / 4.0) {
                    angular.setZ(utils.sign(base_odom_.getTwist().getTwist().getAngular().getZ()) * Math.min(Math.abs(dif_ang / time_to_holo_), max_vel_th_));
                    cmd_vel.setAngular(angular);

                } else {
                    angular.setZ(utils.sign(dif_ang) * Math.min(Math.abs(dif_ang / time_to_holo_), max_vel_th_));
                    cmd_vel.setAngular(angular);

                }
                //ROS_ERROR("vstar = %.3f", vstar);
            } else {
                //new velocity is caculated in world frame, it has to be transformed to robot base frame
                Vector2 rotated_vel = Vector2.rotateVectorByAngle(newVelocity, -position.getHeading());

                linear.setX(rotated_vel.getX());
                linear.setY(rotated_vel.getY());
                cmd_vel.setLinear(linear);
                if (min_dist > 2 * footprint_radius_) {
                    angular.setZ(utils.sign(dif_ang) * Math.min(Math.abs(dif_ang), max_vel_th_));
                    cmd_vel.setAngular(angular);
                }
            }
        } finally {
            me_lock_.unlock();
        }
    }

    //update status of the agent according to its velocity
    void upDateAgentState(ROSAgent agt) {
        double time_dif;
        if (last_seen_ == null)
            time_dif = 0;
        else
            time_dif = node.getCurrentTime().toSeconds() - agt.last_seen_.toSeconds();

        double yaw, x_dif, y_dif, th_dif, x, y, theta;
        Vector2 pt = new Vector2(0.0, 0.0);

        //update position
        yaw = LPutils.getYaw(agt.base_odom_.getPose().getPose().getOrientation());
        th_dif = time_dif * agt.base_odom_.getTwist().getTwist().getAngular().getZ();
        if (agt.holo_robot_) {
            x_dif = time_dif * agt.base_odom_.getTwist().getTwist().getLinear().getX();
            y_dif = time_dif * agt.base_odom_.getTwist().getTwist().getLinear().getY();
        } else {
            x_dif = time_dif * agt.base_odom_.getTwist().getTwist().getLinear().getX() * Math.cos(yaw + th_dif / 2.0);
            y_dif = time_dif * agt.base_odom_.getTwist().getTwist().getLinear().getY() * Math.sin(yaw + th_dif / 2.0);
        }
        theta = yaw + th_dif;
        x = agt.base_odom_.getPose().getPose().getPosition().getX() + x_dif;
        y = agt.base_odom_.getPose().getPose().getPosition().getY() + y_dif;
        agt.position = new Position(x, y, theta);


        agt.timeStep = time_dif;
        //minkowski footprint is in robot frame, in velocity space only need orientation.
        agt.footPrint_rotated = rotateFootprint(agt.minkowski_footprint_, agt.position.getHeading());

        //update velocity
        if (agt.holo_robot_) {
            x = agt.base_odom_.getTwist().getTwist().getLinear().getX();
            y = agt.base_odom_.getTwist().getTwist().getLinear().getY();
            pt.setX(x);
            pt.setY(y);
            agt.velocity = Vector2.rotateVectorByAngle(pt, (yaw + th_dif));
        } else {//?????????????????????????????????????????????????????????????
            double dif_x, dif_y, dif_ang;
            dif_ang = simPeriod * agt.base_odom_.getTwist().getTwist().getAngular().getZ();
            //in robot frame differential robot has only x velocity
            pt.setX(agt.base_odom_.getTwist().getTwist().getLinear().getX() * Math.cos(dif_ang / 2.0));
            pt.setY(agt.base_odom_.getTwist().getTwist().getLinear().getX() * Math.sin(dif_ang / 2.0));
            // in global frame, velocity need no translation
            agt.velocity = Vector2.rotateVectorByAngle(pt, (yaw + th_dif));
        }
    }

    List<Vector2> rotateFootprint(final List<Vector2> footprint, double angle) {
        List<Vector2> result = new ArrayList<Vector2>();
        for (int i = 0; i < footprint.size(); ++i) {
            Vector2 rotated = Vector2.rotateVectorByAngle(footprint.get(i), angle);
            result.add(rotated);
        }
        return result;
    }

    void updateAllNeighbors() {
        neighbors_lock_.lock();
        try {
            for (int i = 0; i < AgentNeighbors.size(); i++) {
                upDateAgentState(AgentNeighbors.get(i));
            }
            NeighborDistComparator comp = new NeighborDistComparator(this);
            AgentNeighbors.sort(comp.getComparator());
        } finally {
            neighbors_lock_.unlock();
        }
    }


    void addNHConstraints(double min_dist, Vector2 pref_velocity) {
        double min_error = minErrorHolo;
        double max_error = maxErrorHolo;
        double error = max_error;
        double v_max_ang = vMaxAng();

        //ROS_ERROR("v_max_ang %.2f", v_max_ang);

        if (min_dist < 2.0 * footprint_radius_ + cur_loc_unc_radius_) {
            error = (max_error - min_error) / (Math.pow(2 * (footprint_radius_ + cur_loc_unc_radius_), 2)) * Math.pow(min_dist, 2) + min_error; // how much error do i allow?
            //ROS_DEBUG("Error = %f", error);
            if (min_dist < 0) {
                error = min_error;
                // ROS_DEBUG("%s I think I am in collision", me_->getId().c_str());
            }
        }
        cur_allowed_error_ = 1.0 / 3.0 * cur_allowed_error_ + 2.0 / 3.0 * error;
        //ROS_ERROR("error = %f", cur_allowed_error_);
        double speed_ang = Math.atan2(pref_velocity.getY(), pref_velocity.getX());
        double dif_ang = Angles.shortest_angular_distance(position.getHeading(), speed_ang);
        //calculate possible tracking holomonic robot speed range
        if (Math.abs(dif_ang) > Math.PI / 2.0) { // || cur_allowed_error_ < 2.0 * min_error) {
            double max_track_speed = NHORCA.calculateMaxTrackSpeedAngle(time_to_holo_, Math.PI / 2.0, cur_allowed_error_, max_vel_x_, max_vel_th_, v_max_ang);
            // tracking errors are not in velocity space!!!!!!!!!!!!!!
            if (max_track_speed <= 2 * min_error) {
                max_track_speed = 2 * min_error;
            }
            NHORCA.addMovementConstraintsDiffSimple(max_track_speed, position.getHeading(), addOrcaLines);
        } else {
            NHORCA.addMovementConstraintsDiff(cur_allowed_error_, time_to_holo_, max_vel_x_, max_vel_th_, position.getHeading(), v_max_ang, addOrcaLines);
        }
        max_speed_x_ = vMaxAng();

    }

    double vMaxAng() {
        //double theoretical_max_v = max_vel_th_ * wheel_base_ / 2.0;
        //return theoretical_max_v - std::abs(base_odom_.twist.twist.angular.z) * wheel_base_/2.0;
        return max_vel_x_; //TODO: fixme
    }


    void computeObstacles() {
        obstacle_lock_.lock();
        try {
            Vector<Vector2> own_footprint = new Vector<Vector2>();

            for (int i = 0; i < footprint_original_msg_.getPolygon().getPoints().size(); i++) {
                own_footprint.add(new Vector2(footprint_original_msg_.getPolygon().getPoints().get(i).getX(),
                        footprint_original_msg_.getPolygon().getPoints().get(i).getY()));
            }

            min_dist_obst_ = Double.MAX_VALUE;
            int i = 0;
            Vector<Integer> delete_list = new Vector<Integer>();
            for (int j = 0; j < obstacles_from_laser_.size(); j++) {
                Obstacle obst = obstacles_from_laser_.get(i);
                if (!obst.getBegin().equals(obst.getEnd())) {
                    double dist = utils.distSqPointLineSegment(obst.getBegin(), obst.getEnd(), position.getPos());
                    //why choose this limit?????????????????????????????????
                    if (dist < Math.pow((Vector2.abs(velocity) + 4.0 * footprint_radius_), 2)) {
                        if (use_obstacles_) {
                            if (orca) {// currently set to false
                                createObstacleLine(own_footprint, obst.getBegin(), obst.getEnd());
                            } else {//called from CP
                                VO obstacle_vo = CP.createObstacleVO(position.getPos(), footprint_radius_, footPrint_rotated, obst.getBegin(), obst.getEnd());
                                voAgents.add(obstacle_vo);
                            }
                        }
                        if (dist < min_dist_obst_) {
                            min_dist_obst_ = dist;
                        }
                    }
                } else {
                    delete_list.add(i);
                }
                i++;
            }

            for (i = delete_list.size() - 1; i >= 0; i--) {
                obstacles_from_laser_.remove(delete_list.get(i));
            }

        } finally {
            obstacle_lock_.unlock();
        }
    }

    void createObstacleLine(Vector<Vector2> own_footprint, Vector2 obst1, Vector2 obst2) {

        double dist = utils.distSqPointLineSegment(obst1, obst2, position.getPos());

        if (dist == Vector2.absSqr(Vector2.minus(position.getPos(), obst1))) {
            computeObstacleLine(obst1);
        } else if (dist == Vector2.absSqr(Vector2.minus(position.getPos(), obst2))) {
            computeObstacleLine(obst2);
        } else {
            Vector2 position_obst = utils.projectPointOnLine(obst1, Vector2.minus(obst2, obst1), position.getPos());
            Vector2 rel_position = Vector2.minus(position_obst, position.getPos());
            dist = Math.sqrt(dist);
            double dist_to_footprint = getDistToFootprint(rel_position);
            if (dist_to_footprint == -1) {
                dist_to_footprint = footprint_radius_;
            }
            dist = dist - dist_to_footprint - 0.03;

            if (dist < 0.0) {
                Line line = new Line();
                line.setPoint(Vector2.mul(Vector2.normalize(rel_position), (dist - 0.02)));
                line.setDir(Vector2.normalize(Vector2.minus(obst1, obst2)));
                addOrcaLines.add(line);
                return;
            }

            if (Vector2.abs(Vector2.minus(position.getPos(), obst1)) > 2 * footprint_radius_ &&
                    Vector2.abs(Vector2.minus(position.getPos(), obst2)) > 2 * footprint_radius_) {
                Line line = new Line();
                line.setPoint(Vector2.mul(Vector2.normalize(rel_position), dist));
                line.setDir(Vector2.negative(Vector2.normalize(Vector2.minus(obst1, obst2))));
                addOrcaLines.add(line);
                return;

            }

            rel_position = Vector2.mul(Vector2.normalize(rel_position), Vector2.abs(rel_position) - dist / 2.0);

            Vector<Vector2> obst = new Vector<Vector2>();
            obst.add(Vector2.minus(obst1, position_obst));
            obst.add(Vector2.minus(obst2, position_obst));
            List<Vector2> mink_sum = CP.minkowskiSumConvexHull(own_footprint, obst);

            Vector2 min = new Vector2();
            Vector2 max = new Vector2();
            double min_ang = 0.0;
            double max_ang = 0.0;

            for (int i = 0; i < mink_sum.size(); i++) {
                double angle = Vector2.angleBetween(rel_position, Vector2.plus(rel_position, mink_sum.get(i)));
                if (utils.leftOf(new Vector2(0.0, 0.0), rel_position, Vector2.plus(rel_position, mink_sum.get(i)))) {
                    if (-angle < min_ang) {
                        min = Vector2.plus(rel_position, mink_sum.get(i));
                        min_ang = -angle;
                    }
                } else {
                    if (angle > max_ang) {
                        max = Vector2.plus(rel_position, mink_sum.get(i));
                        max_ang = angle;
                    }
                }
            }

            Line line = new Line();
            line.setPoint(Vector2.mul(Vector2.normalize(rel_position), dist / 2.0));
            if (Vector2.absSqr(Vector2.minus(position_obst, obst1)) > Vector2.absSqr(Vector2.minus(position_obst, obst2))) {
                // ROS_ERROR("max_ang = %.2f", max_ang);
                line.setDir(Vector2.rotateVectorByAngle(Vector2.normalize(max), 0.1));
            } else {
                // ROS_ERROR("min_ang = %.2f", min_ang);
                line.setDir(Vector2.rotateVectorByAngle(Vector2.normalize(min), 0.1));

            }
            addOrcaLines.add(line);

        }
    }

    void computeObstacleLine(Vector2 obst) {
        Line line = new Line();
        Vector2 relative_position = Vector2.minus(obst, position.getPos());
        double dist_to_footprint;
        double dist = Vector2.abs(Vector2.minus(position.getPos(), obst));
        if (!has_polygon_footprint_)
            dist_to_footprint = footprint_radius_;
        else {
            dist_to_footprint = getDistToFootprint(relative_position);
            if (dist_to_footprint == -1) {
                dist_to_footprint = footprint_radius_;
            }
        }
        dist = dist - dist_to_footprint - 0.03;

        line.setPoint(Vector2.mul(Vector2.normalize(relative_position), dist));
        line.setDir(new Vector2(-(Vector2.normalize(relative_position)).getY(), (Vector2.normalize(relative_position)).getX()));
        addOrcaLines.add(line);
    }

    double getDistToFootprint(Vector2 point) {
        Vector2 result;
        for (int i = 0; i < footprint_original_lines.size(); i++) {
            LinePair l1 = new LinePair(footprint_original_lines.get(i).getFirst(), footprint_original_lines.get(i).getSecond());
            LinePair l2 = new LinePair(new Vector2(0.0, 0.0), point);

            result = LinePair.Intersection(l1, l2);
            if (result != null) {
                //ROS_DEBUG("Result = %f, %f, dist %f", result.x(), result.y(), collvoid::abs(result));
                return Vector2.abs(result);
            }
        }
        //ROS_DEBUG("Obstacle Point within Footprint. I am close to/in collision");
        return -1;
    }


    void computeClearpathVelocity(Vector2 pref_velocity) {
        //account for nh error

        neighbors_lock_.lock();
        try {
            radius += cur_allowed_error_;

            if (controlled) {
                computeAgentVOs();
            }
            newVelocity = CP.calculateClearpathVelocity(samples, voAgents, addOrcaLines, pref_velocity, max_speed_x_, useTruancation);

            radius -= cur_allowed_error_;
        } finally {
            neighbors_lock_.unlock();
        }
        //for visualization
//        msgPublisher.publishHoloSpeed(position, newVelocity, global_frame_, base_frame_, speed_pub_);
//        msgPublisher.publishVOs(position, voAgents, useTruancation, global_frame_, base_frame_, vo_pub_);
//        msgPublisher.publishPoints(position, samples, global_frame_, base_frame_, samples_pub_);
//        msgPublisher.publishOrcaLines(addOrcaLines, position, global_frame_, base_frame_, lines_pub_);


    }

    void computeAgentVOs() {
        // neighbors are published with localization uncertainty that means radius and footprint
        // include localization uncertainty radius and minkowski footprint.
        for (int i = 0; i < AgentNeighbors.size(); i++) {
            VO new_agent_vo;
            //use footprint or radius to create VO
            if (convex) {
                if (AgentNeighbors.get(i).controlled) {
                    new_agent_vo = CP.createVO(position.getPos(), footPrint_rotated, velocity, AgentNeighbors.get(i).position.getPos(), AgentNeighbors.get(i).footPrint_rotated, AgentNeighbors.get(i).velocity, voType);
                } else {
                    new_agent_vo = CP.createVO(position.getPos(), footPrint_rotated, velocity, AgentNeighbors.get(i).position.getPos(), AgentNeighbors.get(i).footPrint_rotated, AgentNeighbors.get(i).velocity, CP.VOS);
                }
            } else {
                if (AgentNeighbors.get(i).controlled) {
                    new_agent_vo = CP.createVO(position.getPos(), radius, velocity, AgentNeighbors.get(i).position.getPos(), AgentNeighbors.get(i).radius, AgentNeighbors.get(i).velocity, voType);
                } else {
                    new_agent_vo = CP.createVO(position.getPos(), radius, velocity, AgentNeighbors.get(i).position.getPos(), AgentNeighbors.get(i).radius, AgentNeighbors.get(i).velocity, CP.VOS);
                }
            }
            //truncation--not collide in certain amount of time
            if (useTruancation) {
                new_agent_vo = CP.createTruncVO(new_agent_vo, truncTime);
            }
            voAgents.add(new_agent_vo);
        }
    }


    /*++++++++++++++++++++++++Get stuff++++++++++++++++++++++++++*/
    public double getRadius() {
        return radius;
    }

    public Odometry_ getBaseOdom() {
        return base_odom_;
    }   

    public long getLastSeen() {
        return last_seen_;
    }

    public String getRobotId() {
        return Name;
    }

    public long getPublishMePeriod() {
        return publishMePeriod;
    }

    public long getLastTimeMePublished() {
        return lastTimeMePublished;
    }

    public RMQMsgManager getRmqMsgManager() {
        return rmqMsgManager;
    }

    public String getBase_frame_() {
        return base_frame_;
    }

    public boolean getHoloRobot() {
        return holo_robot_;
    }

    public boolean getController() {
        return controlled;
    }

    public Vector2 getHolo_velocity_() {
        return holo_velocity_;
    }

    public double getCur_loc_unc_radius_() {
        return cur_loc_unc_radius_;
    }

    public List<Vector2> getFootprint_minkowski() {
        return footprint_minkowski;
    }

    public double getFootprint_radius_() {
        return footprint_radius_;
    }

    public Position getPosition() {
        return position;
    }

    public List<Agent> getAgentNeighbors() {
        return AgentNeighbors;
    }

    public Lock getObstacle_lock_() {
        return obstacle_lock_;
    }

    public Lock getConvex_lock_() {
        return convex_lock_;
    }

    public Lock getMe_lock_() {
        return me_lock_;
    }

    public Lock getNeighbors_lock_() {
        return neighbors_lock_;
    }

    public List<Obstacle> getObstacles_from_laser_() {
        return obstacles_from_laser_;
    }

    public List<Vector2> getFootprint_original() {
        return footprint_original;
    }
/*+++++++++++++++++++++++++Get stuff end+++++++++++++++++++++++++*/

    /*++++++++++++++++++++++++Set stuff++++++++++++++++++++++++++*/

    public void setHolo_robot_(boolean holo_robot_) {
        this.holo_robot_ = holo_robot_;
    }

    public void setHolo_velocity_(Vector2 holo_velocity_) {
        this.holo_velocity_ = holo_velocity_;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public void setControlled(boolean controlled) {
        this.controlled = controlled;
    }

    public void setFootprint_minkowski(List<Vector2> footprint_minkowski) {
        this.footprint_minkowski = footprint_minkowski;
    }

    public void setLast_seen_(long last_seen_) {
        this.last_seen_ = last_seen_;
    }

    public void setLastTimeMePublished(long lastTimeMePublished) {
        this.lastTimeMePublished = lastTimeMePublished;
    }

    // footprint(minkowski footprint) are all in robot frame
    public void setFootprint(List<Vector2> footprint) {
        if (footprint.size() < 2) {
            logger.severe("The footprint specified has less than two nodes");
            return;
        }

        setMinkowskiFootprintVector2(footprint);

        footprint_original_lines.clear();
        Vector2 p = footprint.get(0);
        Vector2 first = new Vector2(p.getX(), p.getY());
        Vector2 old = new Vector2(p.getX(), p.getY());
        //add linesegments for footprint
        for (int i = 0; i < footprint.size(); i++) {
            footprint_original_lines.add(new LinePair(old, footprint.get(i)));
            old.setVector2(footprint.get(i));
        }
        //add last segment
        footprint_original_lines.add(new LinePair(old, first));
        has_polygon_footprint_ = true;
    }


    /*+++++++++++++++++++++++++Set stuff end+++++++++++++++++++++++++*/

}

