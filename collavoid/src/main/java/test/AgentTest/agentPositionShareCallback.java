package test.AgentTest;


import cgl.iotrobots.collavoid.utils.*;
import collavoid_msgs.pose_twist_covariance_msgs;
import costmap_2d.VoxelGrid;
import geometry_msgs.*;
import nav_msgs.Odometry;
import org.ros.concurrent.CancellableLoop;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.*;
import org.ros.namespace.GraphName;
import org.ros.node.*;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.pubsub.TransformListener;
import sensor_msgs.LaserScan;
import sensor_msgs.PointCloud;

import java.net.InetAddress;
import java.net.URI;
import java.net.UnknownHostException;
import java.util.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.regex.PatternSyntaxException;


/**
 * Created by hjh on 11/16/14.
 */
public class agentPositionShareCallback {

        public static TransformListener tf;
        public static VoxelGrid cosmap_voxelGrid;

    public static void main(String[] args)throws InterruptedException{

        String IP = null;
        String hostname = null;

        try {
            InetAddress ia = InetAddress.getLocalHost();
            hostname = ia.getHostName();
            IP = ia.getHostAddress();
        } catch (UnknownHostException e) {
            e.printStackTrace();
        }

        hostname=StringFilter(hostname);//replace all non-alphabet and non-number character into _

        //execute SimplifiedAgent
        NodeConfiguration configuration = NodeConfiguration.newPublic(IP.toString(),
                URI.create("http://localhost:11311"));
        final NodeMainExecutor runner= DefaultNodeMainExecutor.newDefault();
        String agentId=new String("agent_"+hostname);
        configuration.setNodeName(agentId);
        SimplifiedAgent agent=new SimplifiedAgent(agentId,tf);
        runner.execute(agent,configuration);

        while (!SimplifiedAgent.initialized_){
            Thread.sleep(500);
        }
        //execute fakePublisher
        String poseShareId=new String("PoseShare_"+hostname);
        configuration.setNodeName(poseShareId);
        FakePoseSharePub fakePoseSharePub=new FakePoseSharePub();
        runner.execute(fakePoseSharePub,configuration);

    }

    public   static   String StringFilter(String   str)   throws PatternSyntaxException {
        //clear special characters
        String regEx="[`~!@#$%^&*()+=|{}':;',\\[\\].<>/?~！@#￥%……&*（）——+|{}【】‘；：”“’。，、？-]";
        Pattern p   =   Pattern.compile(regEx);
        Matcher m   =   p.matcher(str);
        return   m.replaceAll("_").trim();
    }
}



class SimplifiedAgent extends AbstractNodeMain {
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
    public List<Vector2> footPrint;

    //allowed error for non-holonomic robot
    public double cur_allowed_error_;

    //orca
    public double max_speed_x_; //in nonholomic robot it has only one liner velocity
    public List<Line> orcaLines, addOrcaLines;

    //VO
    public List<VO> voAgents, addVos;
    public List<VelocitySample> samples;

    public List<SimplifiedAgent> agentNeighbors;


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
    public List<obstacle> obstacles_from_laser_;
    public Subscriber laserScanSub, laserNotifier;


    public double min_dist_obst_;

    //obstacles
    public boolean delete_observations_;
    public boolean use_obstacles_;
    public List<Vector2> obstacle_points_;
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
    public static boolean initialized_;
    public boolean has_polygon_footprint_;

    public List<LinePair> footPrintLines;

    public Time last_seen_;
    public Odometry base_odom_;

    //LOC uncertatiny
    public double eps_;
    public double cur_loc_unc_radius_;

    //COLLVOID
    public List<Vector2> minkowski_footprint_;
    public List<PoseWeighted> pose_array_weighted_;

    //lock
    public final Lock neighbors_lock_;

    //me stuff
    TransformListener tf_;

    //subscribers and publishers
    public Publisher lines_pub_, neighbors_pub_, polygon_pub_, vo_pub_, me_pub_, samples_pub_, speed_pub_, position_share_pub_, obstacles_pub_;
    public Subscriber amcl_posearray_sub_, position_share_sub_, odom_sub_, laser_scan_sub_, laser_notifier;


    //utils
    private static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    public static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);

    //to define messages
    public ConnectedNode node;
    public ParameterTree params;


    //-----------------------method begin-------------------------------//

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        this.node=connectedNode;


        //waiting for time
        while (true) {
            try {
                node.getCurrentTime();
                break; // no exception, so let's stop waiting
            } catch (NullPointerException e) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e1) {
                }
            }
        }

        agentInit();
    }

    public SimplifiedAgent(String id,TransformListener tf) {
        this.initialized_=false;

        this.neighbors_lock_ = new ReentrantLock();

//
        this.holo_velocity_=new Vector2();
        this.footprint_msg_=messageFactory.newFromType(PolygonStamped._TYPE);
        this.base_odom_=messageFactory.newFromType(Odometry._TYPE);
//
//        this.minkowski_footprint_=new ArrayList<Vector2>();
//        this.footPrintLines=new ArrayList<LinePair>();
//        this.obstacle_points_=new ArrayList<Vector2>();
//        this.pose_array_weighted_=new ArrayList<PoseWeighted>();
        this.agentNeighbors=new ArrayList<SimplifiedAgent>();
//
//        this.tf_=tf;
        this.id_=new String(id);
    }

    public void agentInit() {

        initPubSub(this.node);
    }


    public void initPubSub(final ConnectedNode newnode) {


        //Subscribers

        position_share_sub_ = node.newSubscriber("/position_share_in", pose_twist_covariance_msgs._TYPE);
        position_share_sub_.addMessageListener(new MessageListener<pose_twist_covariance_msgs>() {
            @Override
            public void onNewMessage(pose_twist_covariance_msgs msg) {
                positionShareCallback(msg);
            }
        }, 10);


        initialized_=true;
//        /*TODO: Need particle cloud and the weights of the particles to calculate localization uncertainty,
//          TODO: but currently just particle cloud.*/
//        amcl_posearray_sub_ = node.newSubscriber("/particlecloud",PoseArray._TYPE);
//        amcl_posearray_sub_.addMessageListener(new MessageListener<PoseArray>() {
//            @Override
//            public void onNewMessage(PoseArray msg) {
//                amclPoseArrayWeightedCallback(msg);
//            }
//        });
//
//
//        //to calculate position
//        odom_sub_=node.newSubscriber("odom",Odometry._TYPE);
//        odom_sub_.addMessageListener(new MessageListener<Odometry>() {
//            @Override
//            public void onNewMessage(Odometry msg) {
//                odomCallback(msg);
//            }
//        });
//
//        laser_scan_sub_ = node.newSubscriber("base_scan", LaserScan._TYPE);
//        laser_scan_sub_.addMessageListener(new MessageListener<LaserScan>() {
//            @Override
//            public void onNewMessage(LaserScan msg) {
//                Duration dur = Duration.fromMillis(100);
//                baseScanCallback(msg, dur);
//
//            }
//        }, 10);


        node.getLog().info("************************Pub and sub initialized");

        while (agentNeighbors.size()<4){
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        for (int i = 0; i <agentNeighbors.size() ; i++) {
            System.out.println("neighbor"+i+" radius is: "+agentNeighbors.get(i).radius);
        }

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

                    SimplifiedAgent new_robot = new SimplifiedAgent(cur_id,null);
                    new_robot.holo_robot_ = msg.getHoloRobot();
                    agentNeighbors.add(new_robot);
                    node.getLog().info("I added a new neighbor with id " + cur_id + " and radius " + msg.getRadius());
                }

                SimplifiedAgent lstagent = agentNeighbors.get(i);
                lstagent.base_odom_.setPose(msg.getPose());
                //TODO Agent -> heading_ = tf::getYaw (msg -> pose.pose.orientation);
                lstagent.base_odom_.setTwist(msg.getTwist());
                lstagent.holo_velocity_ = new Vector2(msg.getHolonomicVelocity().getX(), msg.getHolonomicVelocity().getY());
                lstagent.radius = msg.getRadius();
                lstagent.controlled = msg.getControlled();
                lstagent.footprint_msg_ = msg.getFootprint();
                //lstagent.setMinkowskiFootprintVector2(msg.getFootprint());
                lstagent.last_seen_ = msg.getHeader().getStamp();

            }

//            if ((node.getCurrentTime().toSeconds() - lastTimePositionsPublished.toSeconds()) > publishPositionsPeriod) {
//                lastTimePositionsPublished = node.getCurrentTime();
//                msgPublisher.publishNeighborPositions(agentNeighbors, global_frame_, base_frame_, neighbors_pub_);
//                msgPublisher.publishMePosition(this, global_frame_, base_frame_, me_pub_);
//            }
        } finally {
            neighbors_lock_.unlock();
        }
    }

}

class FakePoseSharePub extends AbstractNodeMain{
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("FakePoseSharePub");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        final Publisher<pose_twist_covariance_msgs> publisher =
                connectedNode.newPublisher("position_share_in", pose_twist_covariance_msgs._TYPE);


        // This CancellableLoop will be canceled automatically when the node shuts
        // down.
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            private int i;

            @Override
            protected void setup() {
                i = 0;
            }

            @Override
            protected void loop() throws InterruptedException {
                if(i<5) {
                    pose_twist_covariance_msgs pmsg =
                            SimplifiedAgent.messageFactory.newFromType(pose_twist_covariance_msgs._TYPE);
                    pmsg.getHeader().setFrameId("base" + i);
                    pmsg.getHeader().setStamp(connectedNode.getCurrentTime());
                    pmsg.setRobotId("SimplifiedAgent_" + i);
                    pmsg.setHoloRobot(false);
                    pmsg.setControlled(true);
                    pmsg.setRadius((float) i);


                    publisher.publish(pmsg);
                    i++;
                }
                Thread.sleep(1000);
            }
        });

    }

}