package cgl.iotrobots.collavoid.ROSAgent;

import cgl.iotrobots.collavoid.ClearPath.CP;
import cgl.iotrobots.collavoid.LocalPlanner.LPutils;
import cgl.iotrobots.collavoid.NHORCA.NHORCA;
import cgl.iotrobots.collavoid.msgmanager.msgPublisher;
import cgl.iotrobots.collavoid.utils.*;

import static cgl.iotrobots.collavoid.NHORCA.NHORCA.calcVstar;
import static cgl.iotrobots.collavoid.utils.utils.*;

import collavoid_msgs.pose_twist_covariance_msgs;
import geometry_msgs.*;
import nav_msgs.Odometry;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.*;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.pubsub.TransformListener;
import sensor_msgs.LaserScan;
import sensor_msgs.PointCloud;

import java.net.UnknownHostException;
import java.net.InetAddress;
import java.util.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Logger;



public class Agent extends AbstractNodeMain {
    //config for simulation and some rules
    double leftPref;
    public boolean useTruancation;
    public double truncTime;
    public double timeStep, simPeriod;

    //whether it is a controlled robot
    public boolean controlled;

    //setting for different CollAvoid strategies
    //orca or vo, need to figure out its usage
    public boolean orca;
    //footprint approximation, use circle approx or mink sum
    public boolean convex;

    //vo setting
    //clearPath or sampling based collAvoid strategy
    boolean clearPath;
    //0:HRVO,1:RVO,2:VO
    public int voType;

    //robot information
    public Position position; //contains the heading information
    public Vector2 velocity;
    public Vector2 newVelocity;
    public double radius;
    //public List<Point<Double>> footPrint;
    public Vector<Vector2> footPrint;

    //allowed error for non-holonomic robot
    public double cur_allowed_error_;

    //orca
    public double max_speed_x_; //in nonholomic robot it has only one liner velocity
    public Vector<Line> orcaLines, addOrcaLines;

    //VO
    public Vector<VO> voAgents, addVos;
    public Vector<VelocitySample> samples;

    public Vector<Agent> agentNeighbors;


    //ROSAgent

    //config
    public double publishPositionsPeriod;
    public double publishMePeriod;

    public double thresholdLastSeen;
    public int numSamples;

    //helpers??
    public Time lastTimePositionsPublished;
    public Time lastTimeMePublished;

    //NH stuff
    public double minErrorHolo;
    public double maxErrorHolo;
    public boolean holo_robot_;
    public double time_to_holo_;

    //ORCA stuff
    public double max_vel_with_obstacles_;
    public Vector2 holo_velocity_;


    //Obstacles
    ///////////////LaserProjection projector_;  //translate laserscan to pointcloud

    //Obstacles
    public Vector<obstacle> obstacles_from_laser_;
    public Subscriber laserScanSub, laserNotifier;


    public double min_dist_obst_;

    //obstacles
    public boolean delete_observations_;
    public boolean use_obstacles_;
    public Vector<Vector2> obstacle_points_;
    public double time_horizon_obst_;

    //Agent description
    public String id_;
    public String base_frame_, global_frame_;
    public double wheel_base_;
    public PolygonStamped footprint_msg_;
    public double acc_lim_x_, acc_lim_y_, acc_lim_th_;
    public double min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, max_vel_th_, min_vel_th_, min_vel_th_inplace_;
    public double footprint_radius_;


    public boolean standalone_;
    //set automatically
    public boolean initialized_;
    public boolean has_polygon_footprint_;

    public List<LinePair> footPrintLines;

    public Time last_seen_;
    public Odometry base_odom_;

    //LOC uncertatiny
    public double eps_;
    public double cur_loc_unc_radius_;

    //COLLVOID
    public List<Vector2> minkowski_footprint_;
    public Vector<poseWeighted> pose_array_weighted_;

    //lock
    public final Lock me_lock_, obstacle_lock_, neighbors_lock_, convex_lock_;

    //me stuff
    TransformListener tf_;

    //subscribers and publishers
    public Publisher lines_pub_, neighbors_pub_, polygon_pub_, vo_pub_, me_pub_, samples_pub_, speed_pub_, position_share_pub_, obstacles_pub_;
    public Subscriber amcl_posearray_sub_, position_share_sub_, odom_sub_, laser_scan_sub_, laser_notifier;


    //utils
    private static Logger logger = Logger.getLogger("Agent");
    public static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    public static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);

    //to define messages
    public ConnectedNode node;
    public ParameterTree params;

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        this.node=connectedNode;
        this.params = node.getParameterTree();
        agentInit();
    }

    //-----------------------method begin-------------------------------//
    public Agent(String id,TransformListener tf) {
        this.me_lock_ = new ReentrantLock();
        this.obstacle_lock_ = new ReentrantLock();
        this.neighbors_lock_ = new ReentrantLock();
        this.convex_lock_ = new ReentrantLock();

        this.initialized_ = false;
        this.cur_allowed_error_ = 0;
        this.cur_loc_unc_radius_ = 0;
        this.min_dist_obst_ = Double.MAX_VALUE;

        this.footprint_msg_=messageFactory.newFromType(PolygonStamped._TYPE);
        this.minkowski_footprint_=new ArrayList<Vector2>();
        this.footPrintLines=new ArrayList<LinePair>();

        this.tf_=tf;
        this.id_=new String(id);
    }

    //get parameters from the server
    public void agentInit() {

        this.base_frame_ = params.getString("~/base_frame", "/base_link");
        this.global_frame_ = params.getString("~/global_frame", "/map");
        this.standalone_ = params.getBoolean("~/standalone", false);

        //acceleration limits load from params_turtle.yaml
        this.acc_lim_x_=params.getDouble("~/acc_lim_x");
        this.acc_lim_y_=params.getDouble("~/acc_lim_y");
        this.acc_lim_th_=params.getDouble("~/acc_lim_th");

        //holo_robot
        this.holo_robot_=params.getBoolean("~/holo_robot");

        if (!holo_robot_)
            this.wheel_base_=params.getDouble("~/wheel_base");
        else
            this.wheel_base_ = 0.0;

        //min max speeds
        this.max_vel_with_obstacles_=params.getDouble("~/max_vel_with_obstacles");

        this.max_vel_x_=params.getDouble("~/max_vel_x");
        this.min_vel_x_=params.getDouble("~/min_vel_x");
        this.max_vel_y_=params.getDouble("~/max_vel_y");
        this.min_vel_y_=params.getDouble("~/min_vel_y");
        this.max_vel_th_=params.getDouble("~/max_vel_th");
        this.min_vel_th_=params.getDouble("~/min_vel_th");
        this.min_vel_th_inplace_=params.getDouble("~/min_vel_th_inplace");

        //set radius
        this.footprint_radius_=params.getDouble("~/footprint_radius");
        this.radius=this.footprint_radius_+this.cur_loc_unc_radius_;

        //set frames
        //robot_base_frame_ = costmap_ros_->getBaseFrameID(); need to investigate
        ////me.base_frame_ = new String(costmap_ros_.getHeader().getFrameId());//may not be right
        ////me.global_frame_ = new String(params.getString("global_frame", "/map"));

        //sim period
        GraphName controller_frequency_param_name;
        controller_frequency_param_name=params.search("~/controller_frequency");
        double sim_period_;
        if(controller_frequency_param_name==null){
            sim_period_ = 0.05;
        }
        else {
            double controller_frequency = 0;
            controller_frequency=params.getDouble(controller_frequency_param_name,20.0);

            if(controller_frequency > 0){
                sim_period_ = 1.0 / controller_frequency;
            }else{
                this.node.getLog().warn("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
                sim_period_ = 0.05;
            }
        }
        this.node.getLog().info("Sim period is set to "+ String.format("%1$.2f",sim_period_));
        this.simPeriod=sim_period_;

        //other params agent
        //me.time_horizon_obst_ = getParamDef(private_nh,"time_horizon_obst",10.0); currently not used in agent
        this.time_to_holo_ = params.getDouble("~/time_to_holo", 0.4);
        this.minErrorHolo = params.getDouble("~/min_error_holo", 0.01);
        this.maxErrorHolo = params.getDouble( "~/max_error_holo", 0.15);
        //delete_observations_ = params.getBoolean("delete_observations", true); currently not used in agent
        //threshold_last_seen_ = params.getDouble("threshold_last_seen",1.0); currently not used in agent
        this.eps_= params.getDouble( "~/eps", 0.1);
        this.orca=params.getBoolean( "~/orca");
        this.convex=params.getBoolean( "~/convex");
        //params.getBoolean( "clearpath", &clearpath); not used
        this.useTruancation=params.getBoolean( "~/use_truncation");

        //num_samples = getParamDef(private_nh, "num_samples", 400); not used
        this.voType = params.getInteger("~/type_vo", 0); //HRVO

        this.truncTime = params.getDouble("~/trunc_time",5.0);
        //left_pref_ = getParamDef(private_nh,"left_pref",0.1); not used

        this.publishPositionsPeriod = 1.0/params.getDouble("~/publish_positions_frequency",10.0);
        this.publishMePeriod = 1.0/params.getDouble("~/publish_me_frequency",10.0);

        //set Footprint
        List<Point> footprint_points=new ArrayList<Point>();
        //TODO:footprint_points = costmap_ros_->getRobotFootprint();

        PolygonStamped footprint=messageFactory.newFromType(PolygonStamped._TYPE);
        Point32 p=messageFactory.newFromType(Point32._TYPE);
        List<Point32> points=new ArrayList<Point32>();
        for (int i = 0; i<footprint_points.size(); i++) {
            p.setX((float)footprint_points.get(i).getX());
            p.setY((float) footprint_points.get(i).getY());
            points.add(p);
        }
        Polygon polygon=messageFactory.newFromType(Polygon._TYPE);
        polygon.setPoints(points);
        footprint.setPolygon(polygon);

        points.clear();
        if (footprint.getPolygon().getPoints().size()>2)
            setFootprint(footprint);
        else {
            double angle = 0;
            double step = 2 * Math.PI / 72;
            while(angle < 2 * Math.PI){
                Point32 pt=messageFactory.newFromType(Point32._TYPE);
                pt.setX((float)(this.footprint_radius_ * Math.cos(angle)));
                pt.setY((float) (this.footprint_radius_ * Math.sin(angle)));
                pt.setZ(0.0f);
                points.add(pt);
                angle += step;
            }
            polygon.setPoints(points);
            footprint.setPolygon(polygon);
            setFootprint(footprint);
        }
/*
        //only coverting round footprints for now
        PolygonStamped footprint = messageFactory.newFromType(PolygonStamped._TYPE);
        Polygon polygon = messageFactory.newFromType(Polygon._TYPE);
        List<Point32> points = new ArrayList<Point32>();

        double angle = 0;
        double step = 2 * Math.PI / 72;
        while (angle < 2 * Math.PI) {
            Point32 pt = messageFactory.newFromType(Point32._TYPE);
            pt.setX((float) (radius * Math.cos(angle)));
            pt.setY((float) (radius * Math.sin(angle)));
            pt.setZ(0.0f);
            points.add(pt);
            angle += step;
        }
        polygon.setPoints(points);
        footprint.setPolygon(polygon);
        setFootprint(footprint);*/

        initialized_ = true;
        logger.info("************************"+node.getName()+" is initialized.");
    }

    //called by local planner, as a module
    public void initAsMe(final ConnectedNode newnode, TransformListener tf) {
        //Params (get from server or set via local_planner)
        tf_ = tf;
        initCommon(newnode, false);
        initialized_ = true;
    }

    public void initCommon(final ConnectedNode newnode, boolean runalone) {
        if (!runalone) {
            this.node = newnode;
            this.params = node.getParameterTree();
            use_obstacles_ = params.getBoolean("move_base/use_obstacles");
            controlled = params.getBoolean("move_base/controlled", true);
        }

        //Publishers
//        vo_pub_ = nh.advertise<visualization_msgs::Marker>("vo", 1);
//        neighbors_pub_ = nh.advertise<visualization_msgs::MarkerArray>("neighbors", 1);
//        me_pub_ = nh.advertise<visualization_msgs::MarkerArray>("me", 1);
//        lines_pub_ = nh.advertise<visualization_msgs::Marker>("orca_lines", 1);
//        samples_pub_ = nh.advertise<visualization_msgs::MarkerArray>("samples", 1);
//        //polygon_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("convex_hull",1);
//        speed_pub_ = nh.advertise<visualization_msgs::Marker>("speed",1);
//        //position_share_pub_ = nh.advertise<collvoid_msgs::PoseTwistWithCovariance>("/position_share_out",1);
//
//        obstacles_pub_ = nh.advertise<visualization_msgs::Marker>("obstacles", 1);

        //need visualization tools
        position_share_pub_ = node.newPublisher("/position_share_out", pose_twist_covariance_msgs._TYPE);
        polygon_pub_ = node.newPublisher("convex_hull", PolygonStamped._TYPE);

        //Subscribers
//        amcl_posearray_sub_ = nh.subscribe("particlecloud_weighted", 1, &ROSAgent::amclPoseArrayWeightedCallback,this);
//        //position_share_sub_ = nh.subscribe("/position_share_in",10, &ROSAgent::positionShareCallback, this);
//        //odom_sub_ = nh.subscribe("odom",1, &ROSAgent::odomCallback, this);

        //need amcl tools
        position_share_sub_ = node.newSubscriber("/position_share_in", pose_twist_covariance_msgs._TYPE);
        position_share_sub_.addMessageListener(new MessageListener<pose_twist_covariance_msgs>() {
            @Override
            public void onNewMessage(pose_twist_covariance_msgs msg) {
                positionShareCallback(msg);
            }
        }, 10);


//        //laser_scan_sub_.subscribe (nh, "base_scan", 1);
//        //laser_notifier.reset(new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, global_frame_, 10));
        laser_scan_sub_ = node.newSubscriber("base_scan", LaserScan._TYPE);
        laser_scan_sub_.addMessageListener(new MessageListener<LaserScan>() {
            @Override
            public void onNewMessage(LaserScan msg) {
                Duration dur = Duration.fromMillis(100);
                baseScanCallback(msg, dur);

            }
        }, 10);
//        laser_notifier->registerCallback(boost::bind(&ROSAgent::baseScanCallback, this, _1));
//        laser_notifier->setTolerance(ros::Duration(0.1));


        logger.info("New Agent as me initialized");

    }


    public void positionShareCallback(final pose_twist_covariance_msgs msg) {

        neighbors_lock_.lock();
        try {

            String cur_id = msg.getRobotId();
            if (!cur_id.equals(id_)) {  //if it is not me do something
                int i;
                for (i = 0; i < agentNeighbors.size(); i++) {

                    //Agent Agent = boost::dynamic_pointer_cast < ROSAgent > (agent_neighbors_[i]);

                    if (agentNeighbors.get(i).id_.equals(cur_id)) {
                        //I found the robot
                        break;
                    }
                }
                if (i >= agentNeighbors.size()) { //Robot is new, so it will be added to the list

                    Agent new_robot = new Agent(cur_id,null);
                    new_robot.holo_robot_ = msg.getHoloRobot();
                    agentNeighbors.add(new_robot);
                    logger.info("I added a new neighbor with id " + cur_id + " and radius " + msg.getRadius());
                }

                Agent lstagent = agentNeighbors.get(i);
                lstagent.base_odom_.setPose(msg.getPose());
                //TODO Agent -> heading_ = tf::getYaw (msg -> pose.pose.orientation);
                lstagent.base_odom_.setTwist(msg.getTwist());
                lstagent.holo_velocity_ = new Vector2(msg.getHolonomicVelocity().getX(), msg.getHolonomicVelocity().getY());
                lstagent.radius = msg.getRadius();
                lstagent.controlled = msg.getControlled();
                lstagent.footprint_msg_ = msg.getFootprint();
                lstagent.setMinkowskiFootprintVector2(msg.getFootprint());
                lstagent.last_seen_ = msg.getHeader().getStamp();

            }

            if ((node.getCurrentTime().toSeconds() - lastTimePositionsPublished.toSeconds()) > publishPositionsPeriod) {
                lastTimePositionsPublished = node.getCurrentTime();
                msgPublisher.publishNeighborPositions(agentNeighbors, global_frame_, base_frame_, neighbors_pub_);
                msgPublisher.publishMePosition(this, global_frame_, base_frame_, me_pub_);
            }
        } finally {
            neighbors_lock_.unlock();
        }
    }

    public void setMinkowskiFootprintVector2(PolygonStamped minkowski_footprint) {
        this.minkowski_footprint_.clear();
        for (int i = 0; i <minkowski_footprint.getPolygon().getPoints().size() ; i++) {
            double x=minkowski_footprint.getPolygon().getPoints().get(i).getX();
            double y=minkowski_footprint.getPolygon().getPoints().get(i).getY();
            this.minkowski_footprint_.add(new Vector2(x,y));
//            System.out.println(i+":"+minkowski_footprint.getPolygon().getPoints().get(i).getX());
//            System.out.println(i+":"+minkowski_footprint.getPolygon().getPoints().get(i).getY());
        }
//        for (Iterator<Point32> it = minkowski_footprint.getPolygon().getPoints().iterator(); it.hasNext(); ) {
//            this.minkowski_footprint_.add(new Vector2(it.next().getX(), it.next().getY()));
//        }

    }

    public void baseScanCallback(final LaserScan msg, Duration dur) {
        PointCloud cloud = messageFactory.newFromType(PointCloud._TYPE);
        //  ROS_ERROR("got cloud");

/*        try {
            tf_->waitForTransform(msg->header.frame_id, global_frame_, msg->header.stamp, ros::Duration(0.3));
            projector_.transformLaserScanToPointCloud(global_frame_, *msg, cloud, *tf_);
        }
        catch (tf::TransformException& e) {
            logger.severe(e.what());
            return;
        }*/


        obstacle_lock_.lock();
        try {

            obstacles_from_laser_.clear();

            double threshold_convex = 0.03;
            double threshold_concave = -0.03;
            //    ROS_ERROR("%d", (int)cloud.points.size());
            Vector2 start;
            List<Point32> points = cloud.getPoints();//TODO: NEED TO DO GET POINTCLOUD DATA
            for (int i = 0; i < points.size(); i++) {
                start = Vector2.Point32ToVector2(points.get(i));
                while (pointInNeighbor(start) && i < points.size()) {
                    i++;
                    start = Vector2.Point32ToVector2(points.get(i));
                }
                if (i == points.size()) {
                    if (!pointInNeighbor(start)) {
                        //obstacles_from_laser_.push_back(std::make_pair(start,start));
                    }
                    return;
                }

                boolean found = false;
                Vector2 prev = start;
                double first_ang = 0;
                double prev_ang = 0;
                Vector2 next;
                while (!found) {
                    i++;
                    if (i == points.size()) {
                        break;
                    }
                    next = Vector2.Point32ToVector2(points.get(i));
                    while (pointInNeighbor(next) && i < points.size()) {
                        i++;
                        next = Vector2.Point32ToVector2(points.get(i));
                    }

                    if (Vector2.abs(Vector2.minus(next, prev)) > 2 * footprint_radius_) {
                        found = true;
                        break;
                    }
                    Vector2 dif = Vector2.minus(next, start);
                    double ang = Math.atan2(dif.getY(), dif.getX());
                    if (!prev.eq(start)) {
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
                    prev = next;
                    prev_ang = ang;
                }
                obstacle obst = new obstacle(start, prev);
                obst.setTime(msg.getHeader().getStamp());

                obstacles_from_laser_.add(obst);
            }
        } finally {
            obstacle_lock_.unlock();
        }
        msgPublisher.publishObstacleLines(obstacles_from_laser_, global_frame_, base_frame_, obstacles_pub_);

    }

    boolean pointInNeighbor(Vector2 point) {
        for (int i = 0; i < agentNeighbors.size(); i++) {
            if (Vector2.abs(Vector2.minus(point, agentNeighbors.get(i).position.getPos())) <= agentNeighbors.get(i).radius)
                return true;
        }
        return false;
    }


    //called by local planner
    /* first refresh agent status according to the last velocity computed and then compute new
    * velocity */
    public void computeNewVelocity(Vector2 pref_velocity, Twist cmd_vel) {
        me_lock_.lock();
        try {
            //Forward project agents,update me status like position and so on
            setAgentParams(this);
            updateAllNeighbors();

            newVelocity = new Vector2(0.0, 0.0);

            addOrcaLines.clear();
            voAgents.clear();

            //get closest Agent/obstacle
            double min_dist_neigh = Double.MAX_VALUE;
            if (agentNeighbors.size() > 0)
                //neighbors have already been sorted according to their dist to me
                min_dist_neigh = Vector2.abs(Vector2.minus(agentNeighbors.get(0).position.getPos(), position.getPos()));

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


            double speed_ang = Math.atan2(newVelocity.getY(),newVelocity.getX());
            double dif_ang = Angles.shortest_angular_distance(position.getHeading(), speed_ang);

            Vector3 linear=messageFactory.newFromType(Vector3._TYPE);
            Vector3 angular=messageFactory.newFromType(Vector3._TYPE);
            if (!holo_robot_) {
                double vel = Vector2.abs(newVelocity);
                double vstar;

                if (Math.abs(dif_ang) > EPSILON.EPSILON)
                vstar = calcVstar(vel, dif_ang);
                else
                vstar = max_vel_x_;

                linear.setX(Math.min(vstar, vMaxAng()));
                linear.setY(0.0);
                cmd_vel.setLinear(linear);

                //ROS_ERROR("dif_ang %f", dif_ang);
                if (Math.abs(dif_ang) > 3.0 * Math.PI / 4.0){
                    angular.setZ(sign(base_odom_.getTwist().getTwist().getAngular().getZ()) * Math.min(Math.abs(dif_ang / time_to_holo_), max_vel_th_));
                    cmd_vel.setAngular(angular);

                }
                else{
                    angular.setZ(sign(dif_ang) * Math.min(Math.abs(dif_ang / time_to_holo_), max_vel_th_));
                    cmd_vel.setAngular(angular);

                }
                //ROS_ERROR("vstar = %.3f", vstar);
            } else {
                Vector2 rotated_vel = Vector2.rotateVectorByAngle(newVelocity, -position.getHeading());

                linear.setX(rotated_vel.getX());
                linear.setY(rotated_vel.getY());
                cmd_vel.setLinear(linear);
                if (min_dist > 2 * footprint_radius_) {
                    angular.setZ(sign(dif_ang) * Math.min(Math.abs(dif_ang), max_vel_th_));
                    cmd_vel.setAngular(angular);
                }
            }
        } finally {
            me_lock_.unlock();
        }
    }

    //update status of the agent according to its velocity
    void setAgentParams(Agent agt) {
        double time_dif = agt.node.getCurrentTime().toSeconds() - agt.last_seen_.toSeconds();
        double yaw, x_dif, y_dif, th_dif, x, y, theta;
        Vector2 pt = new Vector2(0.0, 0.0);


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

        agt.footPrint = rotateFootprint(agt.minkowski_footprint_, agt.position.getHeading());

        if (agt.holo_robot_) {
            x = agt.base_odom_.getPose().getPose().getPosition().getX() + x_dif;
            y = agt.base_odom_.getPose().getPose().getPosition().getY() + y_dif;
            pt.setX(x);
            pt.setY(y);
            agt.velocity = Vector2.rotateVectorByAngle(pt, (yaw + th_dif));
        } else {
            double dif_x, dif_y, dif_ang;
            dif_ang = simPeriod * agt.base_odom_.getTwist().getTwist().getAngular().getZ();
            pt.setX(agt.base_odom_.getPose().getPose().getPosition().getX() * Math.cos(dif_ang / 2.0));
            pt.setY(agt.base_odom_.getPose().getPose().getPosition().getX() * Math.sin(dif_ang / 2.0));
            agt.velocity = Vector2.rotateVectorByAngle(pt, (yaw + th_dif));
        }
    }

    Vector<Vector2> rotateFootprint(final List<Vector2> footprint, double angle) {
        Vector<Vector2> result = new Vector<Vector2>();
        for (int i = 0; i < footprint.size(); ++i) {
            Vector2 rotated = Vector2.rotateVectorByAngle(footprint.get(i), angle);
            result.add(rotated);
        }
        return result;
    }

    void updateAllNeighbors() {
        neighbors_lock_.lock();
        try {
            for (int i = 0; i < agentNeighbors.size(); i++) {
                setAgentParams(agentNeighbors.get(i));
            }
            NeighborDistComparator comp = new NeighborDistComparator(this);
            agentNeighbors.sort(comp.getComparator());
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
        return max_vel_x_; //TODO: fixme!!?????
    }


    void computeObstacles() {
        obstacle_lock_.lock();
        try {
            Vector<Vector2> own_footprint = new Vector<Vector2>();

            for (Iterator<Point32> it = footprint_msg_.getPolygon().getPoints().iterator(); it.hasNext(); ) {
                own_footprint.add(new Vector2(it.next().getX(), it.next().getY()));
                //      ROS_WARN("footprint point p = (%f, %f) ", footprint_[i].x, footprint_[i].y);
            }

            min_dist_obst_ = Double.MIN_VALUE;
            Time cur_time = node.getCurrentTime();
            int i = 0;
            Vector<Integer> delete_list = new Vector<Integer>();
            for (Iterator<obstacle> obst = obstacles_from_laser_.iterator(); obst.hasNext(); ) {
                if (!obst.next().getBegin().eq(obst.next().getEnd())) {
                    double dist = distSqPointLineSegment(obst.next().getBegin(), obst.next().getEnd(), position.getPos());
                    if (dist < Math.pow((Vector2.abs(velocity) + 4.0 * footprint_radius_), 2)) {
                        if (use_obstacles_) {
                            if (orca) {
                                createObstacleLine(own_footprint, obst.next().getBegin(), obst.next().getEnd());
                            } else {//called from CP
                                VO obstacle_vo = CP.createObstacleVO(position.getPos(), footprint_radius_, own_footprint, obst.next().getBegin(), obst.next().getEnd());
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

        double dist = distSqPointLineSegment(obst1, obst2, position.getPos());

        if (dist == Vector2.absSqr(Vector2.minus(position.getPos(), obst1))) {
            computeObstacleLine(obst1);
        } else if (dist == Vector2.absSqr(Vector2.minus(position.getPos(), obst2))) {
            computeObstacleLine(obst2);
        }else {
            Vector2 position_obst = projectPointOnLine(obst1, Vector2.minus(obst2,obst1), position.getPos());
            Vector2 rel_position = Vector2.minus(position_obst,position.getPos());
            dist = Math.sqrt(dist);
            double dist_to_footprint = getDistToFootprint(rel_position);
            if (dist_to_footprint == -1) {
                dist_to_footprint = footprint_radius_;
            }
            dist = dist - dist_to_footprint - 0.03;

            if (dist < 0.0) {
                Line line=new Line();
                line.setPoint(Vector2.mul(Vector2.normalize(rel_position),(dist - 0.02)));
                line.setDir(Vector2.normalize(Vector2.minus(obst1, obst2)));
                addOrcaLines.add(line);
                return;
            }

            if (Vector2.abs(Vector2.minus(position.getPos(),obst1)) > 2 * footprint_radius_ &&
                    Vector2.abs(Vector2.minus(position.getPos(),obst2)) > 2 * footprint_radius_) {
                Line line=new Line();
                line.setPoint(Vector2.mul(Vector2.normalize(rel_position),dist));
                line.setDir(Vector2.negative(Vector2.normalize(Vector2.minus(obst1, obst2))));
                addOrcaLines.add(line);
                return;

            }

            rel_position = Vector2.mul(Vector2.normalize(rel_position),Vector2.abs(rel_position) - dist / 2.0);

            Vector<Vector2> obst=new Vector<Vector2>();
            obst.add(Vector2.minus(obst1, position_obst));
            obst.add(Vector2.minus(obst2,position_obst));
            List<Vector2> mink_sum = CP.minkowskiSum(own_footprint, obst);

            Vector2 min=new Vector2();
            Vector2 max=new Vector2();
            double min_ang = 0.0;
            double max_ang = 0.0;

            for (int i = 0; i < mink_sum.size(); i++) {
                double angle = Vector2.angleBetween(rel_position, Vector2.plus(rel_position,mink_sum.get(i)));
                if (leftOf(new Vector2(0.0, 0.0), rel_position, Vector2.plus(rel_position,mink_sum.get(i)))) {
                    if (-angle < min_ang) {
                        min = Vector2.plus(rel_position,mink_sum.get(i));
                        min_ang = -angle;
                    }
                } else {
                    if (angle > max_ang) {
                        max = Vector2.plus(rel_position,mink_sum.get(i));
                        max_ang = angle;
                    }
                }
            }

            Line line=new Line();
            line.setPoint(Vector2.mul(Vector2.normalize(rel_position), dist / 2.0));
            if (Vector2.absSqr(Vector2.minus(position_obst,obst1)) > Vector2.absSqr(Vector2.minus(position_obst, obst2))) {
                // ROS_ERROR("max_ang = %.2f", max_ang);
                line.setDir(Vector2.rotateVectorByAngle(Vector2.normalize(max), 0.1));
            } else {
                // ROS_ERROR("min_ang = %.2f", min_ang);
                line.setDir(Vector2.rotateVectorByAngle(Vector2.normalize(min), 0.1));

            }
            addOrcaLines.add(line);

        }
    }

    void computeObstacleLine(Vector2 obst){
        Line line=new Line();
        Vector2 relative_position = Vector2.minus(obst,position.getPos());
        double dist_to_footprint;
        double dist = Vector2.abs(Vector2.minus(position.getPos(), obst));
        if (!has_polygon_footprint_)
            dist_to_footprint = footprint_radius_;
        else {
            dist_to_footprint = getDistToFootprint(relative_position);
            if (dist_to_footprint == -1){
                dist_to_footprint = footprint_radius_;
            }
        }
        dist  = dist - dist_to_footprint - 0.03;

        line.setPoint(Vector2.normalize(Vector2.mul(relative_position, dist)));
        line.setDir(new Vector2(-(Vector2.normalize(relative_position)).getY(),(Vector2.normalize(relative_position)).getX())) ;
        addOrcaLines.add(line);
    }

    double getDistToFootprint(Vector2 point){
        Vector2 result;
        for (int i = 0; i < footPrintLines.size(); i++){
            LinePair l1=new LinePair(footPrintLines.get(i).getFirst(),footPrintLines.get(i).getSecond());
            LinePair l2=new LinePair(new Vector2(0.0,0.0),point);

            result = LinePair.Intersection(l1,l2);
            if (result!= null) {
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
        }finally {
            neighbors_lock_.unlock();
        }
        //TODO:ADD PUBLISERS
//            publishHoloSpeed(position_, new_velocity_, global_frame_, base_frame_, speed_pub_);
//            publishVOs(position_, vo_agents_, use_truncation_, global_frame_, base_frame_, vo_pub_);
//            publishPoints(position_, samples_, global_frame_, base_frame_, samples_pub_);
//            publishOrcaLines(additional_orca_lines_, position_, global_frame_, base_frame_, lines_pub_);


    }

    void computeAgentVOs(){

        for (Iterator<Agent> agent = agentNeighbors.iterator(); agent.hasNext(); ) {
            VO new_agent_vo;
            //use footprint or radius to create VO
            if (convex) {
                if (agent.next().controlled) {
                    new_agent_vo = CP.createVO(position.getPos(), footPrint, velocity, agent.next().position.getPos(), agent.next().footPrint, agent.next().velocity, voType);
                } else {
                    new_agent_vo = CP.createVO(position.getPos(), footPrint, velocity, agent.next().position.getPos(), agent.next().footPrint, agent.next().velocity, CP.VOS);
                                    }
            } else {
                if (agent.next().controlled) {
                    new_agent_vo = CP.createVO(position.getPos(), radius, velocity, agent.next().position.getPos(), agent.next().radius, agent.next().velocity, voType);
                } else {
                    new_agent_vo = CP.createVO(position.getPos(), radius, velocity, agent.next().position.getPos(), agent.next().radius, agent.next().velocity, CP.VOS);
                }
            }
            //truncate
            if (useTruancation) {
                new_agent_vo = CP.createTruncVO(new_agent_vo, truncTime);
            }
            voAgents.add(new_agent_vo);
        }
    }





    /*++++++++++++++++++++++++Get and set stuff++++++++++++++++++++++++++*/
    public Odometry getBaseOdom(){
        Odometry odom=messageFactory.newFromType(Odometry._TYPE);
        odom.setTwist(this.base_odom_.getTwist());
        odom.setHeader(this.base_odom_.getHeader());
        odom.setPose(this.base_odom_.getPose());
        odom.setChildFrameId(this.base_odom_.getChildFrameId());
        return odom;
    }

    public void setFootprint(PolygonStamped footprint) {
        if (footprint.getPolygon().getPoints().size() < 2) {
            logger.severe("The footprint specified has less than two nodes");
            return;
        }
        footprint_msg_.setHeader(footprint.getHeader());
        footprint_msg_.setPolygon(footprint.getPolygon());

        setMinkowskiFootprintVector2(footprint_msg_);

        footPrintLines.clear();
        Point32 p = footprint_msg_.getPolygon().getPoints().get(0);
        Vector2 first = new Vector2(p.getX(), p.getY());
        Vector2 old = new Vector2(p.getX(), p.getY());
        Vector2 point = new Vector2(0.0, 0.0);
        //add linesegments for footprint
        for (int i = 0; i < footprint_msg_.getPolygon().getPoints().size(); i++) {
            p = footprint_msg_.getPolygon().getPoints().get(i);
            point.setX(p.getX());
            point.setY(p.getY());
            footPrintLines.add(new LinePair(old,point));
            old.setVector2(point);
        }
        //add last segment
        footPrintLines.add(new LinePair(old,first));
        has_polygon_footprint_ = true;
    }


    /*+++++++++++++++++++++++++Get and set stuff end+++++++++++++++++++++++++*/

}
