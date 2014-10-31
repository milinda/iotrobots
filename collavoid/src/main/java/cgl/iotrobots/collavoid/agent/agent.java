package cgl.iotrobots.collavoid.agent;

/**
 * Created by hjh on 10/24/14.
 */
import cgl.iotrobots.collavoid.msgmanager.msgPublisher;
import cgl.iotrobots.collavoid.utils.*;
import collavoid_msgs.pose_twist_covariance_msgs;
import geometry_msgs.*;
import nav_msgs.Odometry;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.RawMessage;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.*;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.pubsub.TransformListener;
import sensor_msgs.LaserScan;
import sensor_msgs.PointCloud;

import java.net.UnknownHostException;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Logger;

public class agent {
    //config for simulation and some rules
    double leftPref;
    boolean useTruancation;
    double truncTime;
    double timeStep, simPeriod;

    //whether it is a controlled robot
    boolean controlled;

    //setting for different CollAvoid strategies
    //orca or vo, need to figure out its usage
    boolean orca;
    //footprint approximation, use circle approx or mink sum
    boolean convex;

    //vo setting
    //clearPath or sampling based collAvoid strategy
    boolean clearPath;
    //0:HRVO,1:RVO,2:VO
    int voType;

    //robot information
    public Point3<Double> position;
    public Point2<Double> velocity;
    public Point2<Double> newVelocity;
    public double radius;
    //public List<Point<Double>> footPrint;
    public Vector<Point2<Double>> footPrint;

    //allowed error for non-holonomic robot
    public double cur_allowed_error_;

    //orca
    public double maxSpeedX;
    public Vector<Line<Double>> orcaLines, addOrcaLines;

    //VO
    public Vector<VO> voAgents, addVos;
    public Vector<velocitySample> samples;

    public Vector<agent> agentNeighbors;


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
    public Point2<Double> holo_velocity_;


    //Obstacles
    ///////////////LaserProjection projector_;  //translate laserscan to pointcloud

    //Obstacles
    public Vector<obstacle> obstacles_from_laser_;
    public Subscriber laserScanSub, laserNotifier;


    public double min_dist_obst_;

    //obstacles
    public boolean delete_observations_;
    public boolean use_obstacles_;
    public List<Point2<Double>> obstacle_points_;
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

    public Vector<LinePair> footPrintLines;

    public Time last_seen_;
    Odometry base_odom_;

    //LOC uncertatiny
    public double eps_;
    public double cur_loc_unc_radius_;

    //COLLVOID
    public Vector<Point2<Double>> minkowski_footprint_;
    public Vector<poseWeighted> pose_array_weighted_;

    private final Lock me_lock_, obstacle_lock_, neighbors_lock_, convex_lock_;

    //me stuff
    TransformListener tf_;

    //subscribers and publishers
    public Publisher lines_pub_, neighbors_pub_, polygon_pub_, vo_pub_, me_pub_, samples_pub_, speed_pub_, position_share_pub_, obstacles_pub_;
    public Subscriber amcl_posearray_sub_, position_share_sub_, odom_sub_,laser_scan_sub_,laser_notifier;


    //utils
    private static Logger logger = Logger.getLogger("agent");

    //to define messages
    public static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    public static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);

//-----------------------method begin-------------------------------//
    public agent() {
        this.me_lock_ = new ReentrantLock();
        this.obstacle_lock_ = new ReentrantLock();
        this.neighbors_lock_ = new ReentrantLock();
        this.convex_lock_ = new ReentrantLock();

        initialized_ = false;
        cur_allowed_error_ = 0;
        cur_loc_unc_radius_ = 0;
        min_dist_obst_ = Double.MAX_VALUE;
    }

//for standing along running
    public void agentInit(ConnectedNode node, TransformListener tf) {
        tf_ = tf;
        ParameterTree params = node.getParameterTree();
        base_frame_ = params.getString("base_frame", "/base_link");
        global_frame_ = params.getString("global_frame", "/map");
        standalone_ = params.getBoolean("standalone", false);

        String IP = null;
        String hostname = null;

        try {
            InetAddress ia = InetAddress.getLocalHost();
            hostname = ia.getHostName();//获取计算机主机名
            IP = ia.getHostAddress();//获取计算机IP
        } catch (UnknownHostException e) {
            e.printStackTrace();
        }

        //System.out.println(host);
        //System.out.println(IP);


        //if (standalone_) {
        //    ConnectedNode selfNode;
            id_ = node.getName().toString();
            if (id_.equals("/")) {
                id_ = hostname;
            }
            logger.info("Standalone My name is: " + id_);
            controlled = false;
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            initCommon(node);
       // } else {
       //     id_ = private_nh.getNamespace();
        //    if (strcmp(id_.c_str(), "/") == 0) {
        //        char hostname[ 1024];
        //        hostname[1023] = '\0';
        //        gethostname(hostname, 1023);
        //        id_ = std::string (hostname);
        //    }
        //    logger.info("My name is: "+id_);
         //   controlled_ = false;
          //  initCommon(private_nh);
        //}


        radius = footprint_radius_;
        //only coverting round footprints for now
        PolygonStamped footprint=messageFactory.newFromType(PolygonStamped._TYPE);
        List points= new ArrayList();
        Polygon polygon=messageFactory.newFromType(Polygon._TYPE);
        double angle = 0;
        double step = 2 * Math.PI/72;
        while(angle < 2 * Math.PI){
            Point32 pt = messageFactory.newFromType(Point32._TYPE);
            pt.setX((float)(radius*Math.cos(angle)));
            pt.setY((float) (radius * Math.sin(angle)));
            pt.setZ(0.0f);
            points.add(pt);
            angle += step;
        }
        polygon.setPoints(points);
        footprint.setPolygon(polygon);
        setFootprint(footprint);

        eps_= params.getDouble("eps", 0.1);
        convex=params.getBoolean("convex");
        holo_robot_=params.getBoolean("holo_robot");

        publishPositionsPeriod = params.getDouble("publish_positions_frequency",10.0);
        publishPositionsPeriod = 1.0 / publishPositionsPeriod;

        publishMePeriod = params.getDouble("publish_me_frequency",10.0);
        publishMePeriod = 1.0/publishMePeriod;

//        if (standalone_) {
//            //initParams(private_nh);
//        }

        initialized_ = true;
    }

//called by local planner, as a module
    public void initAsMe(ConnectedNode node,TransformListener tf){
        //Params (get from server or set via local_planner)
        tf_ = tf;
        ParameterTree params=node.getParameterTree();
        use_obstacles_=params.getBoolean("move_base/use_obstacles");
        controlled = params.getBoolean("move_base/controlled", true);
        initCommon(node);
        initialized_ = true;
    }

    public void initCommon(final ConnectedNode node){
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
        position_share_pub_ = node.newPublisher("/position_share_out",pose_twist_covariance_msgs._TYPE);
        polygon_pub_ = node.newPublisher("convex_hull", PolygonStamped._TYPE);

        //Subscribers
//        amcl_posearray_sub_ = nh.subscribe("particlecloud_weighted", 1, &ROSAgent::amclPoseArrayWeightedCallback,this);
//        //position_share_sub_ = nh.subscribe("/position_share_in",10, &ROSAgent::positionShareCallback, this);
//        //odom_sub_ = nh.subscribe("odom",1, &ROSAgent::odomCallback, this);

        //need amcl tools
        position_share_sub_ = node.newSubscriber("/position_share_in",pose_twist_covariance_msgs._TYPE);
        position_share_sub_.addMessageListener(new MessageListener<pose_twist_covariance_msgs>() {
            @Override
            public void onNewMessage(pose_twist_covariance_msgs msg) {
                positionShareCallback(msg, node);
            }
        }, 10);


//        //laser_scan_sub_.subscribe (nh, "base_scan", 1);
//        //laser_notifier.reset(new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, global_frame_, 10));
        laser_scan_sub_=node.newSubscriber("base_scan", LaserScan._TYPE);
        laser_scan_sub_.addMessageListener(new MessageListener<LaserScan>() {
            @Override
            public void onNewMessage(LaserScan msg) {
                Duration dur=Duration.fromMillis(100);
                baseScanCallback(msg,dur);

            }
        },10);
//        laser_notifier->registerCallback(boost::bind(&ROSAgent::baseScanCallback, this, _1));
//        laser_notifier->setTolerance(ros::Duration(0.1));




        logger.info("New Agent as me initialized");

    }


    public void positionShareCallback(final pose_twist_covariance_msgs msg,final ConnectedNode node){

        neighbors_lock_.lock();
        try {

            String cur_id = msg.getRobotId();
            if (!cur_id.equals(id_)) {  //if it is not me do something
                int i;
                for (i = 0; i < agentNeighbors.size(); i++) {

                    //agent agent = boost::dynamic_pointer_cast < ROSAgent > (agent_neighbors_[i]);

                    if (agentNeighbors.get(i).id_.equals(cur_id)) {
                        //I found the robot
                        break;
                    }
                }
                if (i >= agentNeighbors.size()) { //Robot is new, so it will be added to the list

                    agent new_robot = new agent();
                    new_robot.id_ = cur_id;
                    new_robot.holo_robot_ = msg.getHoloRobot();
                    agentNeighbors.add(new_robot);
                    logger.info("I added a new neighbor with id "+cur_id+" and radius "+msg.getRadius());
                }

                agent lstagent=agentNeighbors.get(i);
                lstagent.base_odom_.setPose(msg.getPose());
                //TODO agent -> heading_ = tf::getYaw (msg -> pose.pose.orientation);
                lstagent.base_odom_.setTwist(msg.getTwist());
                lstagent.holo_velocity_=new Point2<Double>(msg.getHolonomicVelocity().getX(),msg.getHolonomicVelocity().getY());
                lstagent.radius=msg.getRadius();
                lstagent.controlled = msg.getControlled();
                lstagent.footprint_msg_=msg.getFootprint();
                lstagent.setMinkowskiFootprintVector2(msg.getFootprint());
                lstagent.last_seen_=msg.getHeader().getStamp();

            }

            if ((node.getCurrentTime().toSeconds()-lastTimePositionsPublished.toSeconds()) > publishPositionsPeriod){
                lastTimePositionsPublished = node.getCurrentTime();
                msgPublisher.publishNeighborPositions(agentNeighbors, global_frame_, base_frame_, neighbors_pub_);
                msgPublisher.publishMePosition(this, global_frame_, base_frame_, me_pub_);
            }
        }finally {
            neighbors_lock_.unlock();
        }
    }

    public void setMinkowskiFootprintVector2(PolygonStamped minkowski_footprint) {
        minkowski_footprint_.clear();
        for(Iterator<Point32> it=minkowski_footprint.getPolygon().getPoints().iterator(); it.hasNext();){
            Double x=new Double(it.next().getX());
            Double y=new Double(it.next().getY());
            minkowski_footprint_.addElement(new Point2<Double>(x,y));
        }

    }

    public void baseScanCallback(final LaserScan msg,Duration dur) {
        PointCloud cloud=messageFactory.newFromType(PointCloud._TYPE);
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
            Point2<Double> start;
            List<Point32> points=cloud.getPoints();
            for (int i = 0; i < (int) points.size(); i++) {
                start=Point2.Point32ToPoint2(points.get(i));
                while (pointInNeighbor(start) && i < (int) points.size()) {
                    i++;
                    start = Point2.Point32ToPoint2(points.get(i));
                }
                if (i == (int) points.size()) {
                    if (!pointInNeighbor(start)) {
                        //obstacles_from_laser_.push_back(std::make_pair(start,start));
                    }
                    return;
                }

                boolean found = false;
                Point2<Double> prev = start;
                double first_ang = 0;
                double prev_ang = 0;
                Point2<Double> next;
                while (!found) {
                    i++;
                    if (i == (int) points.size()) {
                        break;
                    }
                    next = Point2.Point32ToPoint2(points.get(i));
                    while (pointInNeighbor(next) && i < (int) points.size()) {
                        i++;
                        next = Point2.Point32ToPoint2(points.get(i));
                    }

                    if (Point2.abs(Point2.minus(next,prev)) > 2 * footprint_radius_) {
                        found = true;
                        break;
                    }
                    Point2<Double> dif = Point2.minus(next,start);
                    double ang = Math.atan2(dif.y, dif.x);
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
                    prev = next;
                    prev_ang = ang;
                }
                obstacle obst=new obstacle(start,prev);
                obst.lastSeen = msg.getHeader().getStamp();

                obstacles_from_laser_.add(obst);
            }
        }finally
        {
            obstacle_lock_.unlock();
        }
        msgPublisher.publishObstacleLines(obstacles_from_laser_, global_frame_, base_frame_, obstacles_pub_);

    }

    boolean pointInNeighbor(Point2<Double> point) {
        for (int i = 0; i<agentNeighbors.size();i++){
            if (Point2.abs(Point2.minus(point,agentNeighbors.get(i).position.getPoint2())) <= agentNeighbors.get(i).radius)
            return true;
        }
        return false;
    }

    void setFootprint(PolygonStamped footprint ){
        if (footprint.getPolygon().getPoints().size() < 2) {
            logger.severe("The footprint specified has less than two nodes");
            return;
        }
        footprint_msg_ = footprint;
        setMinkowskiFootprintVector2(footprint_msg_);

        footPrintLines.clear();
        Point32 p = footprint_msg_.getPolygon().getPoints().get(0);
        Point2 first = new Point2(p.getX(), p.getY());
        Point2 old = new Point2(p.getX(), p.getY());
        Point2 point=new Point2(0.0,0.0);
        LinePair line=new LinePair();
        //add linesegments for footprint
        for (int i = 0; i<footprint_msg_.getPolygon().getPoints().size(); i++) {
            p = footprint_msg_.getPolygon().getPoints().get(i);
            point.x=p.getX();
            point.y=p.getY();
            line.first=old;
            line.second=point;
            footPrintLines.add(line);
            old = point;
        }
        //add last segment
        line.first=old;
        line.second=first;
        footPrintLines.add(line);
        has_polygon_footprint_ = true;
    }



    void setAgentParams(ConnectedNode node, agent agt) {
        double time_dif = node.getCurrentTime().toSeconds() - agt.last_seen_.toSeconds();
        double yaw, x_dif, y_dif, th_dif,x,y,theta;
        Point2<Double> pt=new Point2<Double>(0.0,0.0);
        //time_dif = 0.0;

        yaw = getYaw(agt.base_odom_.getPose().getPose().getOrientation());
        th_dif =  time_dif * agt.base_odom_.getTwist().getTwist().getAngular().getZ();
        if (agt.holo_robot_) {
            x_dif = time_dif * agt.base_odom_.getTwist().getTwist().getLinear().getX();
            y_dif = time_dif * agt.base_odom_.getTwist().getTwist().getLinear().getY();
        }
        else {
            x_dif = time_dif * agt.base_odom_.getTwist().getTwist().getLinear().getX() * Math.cos(yaw + th_dif / 2.0);
            y_dif = time_dif * agt.base_odom_.getTwist().getTwist().getLinear().getY() * Math.sin(yaw + th_dif / 2.0);
        }
        theta=yaw+th_dif;
        x=agt.base_odom_.getPose().getPose().getPosition().getX() + x_dif;
        y=agt.base_odom_.getPose().getPose().getPosition().getY() + y_dif;
        agt.position=new Point3<Double>(x,y,theta);

        //agent->last_seen_ = ros::Time::now();
        agt.timeStep=time_dif;


        //agent->footprint_ = agent->minkowski_footprint_;
        agt.footPrint = rotateFootprint(agt.minkowski_footprint_, agt.position.theta);

        if (agt.holo_robot_) {
            x=agt.base_odom_.getPose().getPose().getPosition().getX() + x_dif;
            y=agt.base_odom_.getPose().getPose().getPosition().getY() + y_dif;
            pt.x=x;
            pt.y=y;
            agt.velocity = Point2.rotateVectorByAngle(pt, (yaw+th_dif));
        }
        else {
            double dif_x, dif_y, dif_ang;
            dif_ang = simPeriod * agt.base_odom_.getTwist().getTwist().getAngular().getZ();
            pt.x = agt.base_odom_.getPose().getPose().getPosition().getX() * Math.cos(dif_ang / 2.0);
            pt.y = agt.base_odom_.getPose().getPose().getPosition().getX() * Math.sin(dif_ang / 2.0);
            agt.velocity = Point2.rotateVectorByAngle(pt, (yaw + th_dif));
        }
    }

    double getYaw(Quaternion quaternion){
        //TODO: get yaw from quaternion
        return 0;

    }

    Vector<Point2<Double>> rotateFootprint(final Vector<Point2<Double>> footprint, double angle) {
        Vector<Point2<Double>> result=new Vector<Point2<Double>>();
        for(int i=0;i<footprint.size();++i){
            Point2<Double> rotated=Point2.rotateVectorByAngle(footprint.get(i), angle);
            result.add(rotated);
        }
        return result;
    }

    void updateAllNeighbors(ConnectedNode node){
        neighbors_lock_.unlock();
        try{
            for (int i = 0; i < agentNeighbors.size(); i++) {
                setAgentParams(node,agentNeighbors.get(i));
                }
        NeighborDistComparator comp=new NeighborDistComparator(this);
            agentNeighbors.sort(comp.getComparator());
    }finally {
            neighbors_lock_.unlock();
        }
    }

}
