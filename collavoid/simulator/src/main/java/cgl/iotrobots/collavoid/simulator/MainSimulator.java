package cgl.iotrobots.collavoid.simulator;

import cgl.iotrobots.collavoid.commons.planners.Parameters;
import cgl.iotrobots.collavoid.controller.AgentController;
import com.rabbitmq.client.Address;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.PoseStamped;
import geometry_msgs.Twist;
import nav_msgs.Odometry;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.pubsub.TransformBroadcaster;
import sensor_msgs.PointCloud2;
import simbad.gui.Simbad;
import simbad.sim.*;

import javax.media.j3d.Transform3D;
import javax.vecmath.*;
import java.util.ArrayList;
import java.util.List;


public class MainSimulator {
    static final int robotNb = Parameters.ROBOT_NUMBER;
    static final double posRadius = Parameters.POSE_RADIUS;
    static List<Robot> robots = new ArrayList<Robot>();

    /**
     * Describe the robot
     */
    static public class Robot extends Agent {

        //id
        int id;

        //for shutdown
        AgentNode agentNode;

        //orientation
        double orientation;
        //kinematic
        DifferentialKinematic kinematic;
        private double wheelDistance;

        //sensors
        LaserScan laserScan;
        RangeSensorBelt sensors;

        //node
        private ConnectedNode node;
        private ParameterTree params;

        //start and goal pubisher
        Publisher<PoseStamped> startGoalPublisher = null;
        PoseStamped startGoalMsg;
        Pose start, goal;
        int sgseq;
        
        //odometry publisher
        Publisher<Odometry> odometryPublisher = null;
        Odometry odomMsg;
        int odomSeq;

        //laser scan publisher
        Publisher<PointCloud2> laserscanPublisher = null;
        PointCloud2 pc2,pctmp;
        int pc2Seq;

        //velocity command subscriber
        Subscriber<Twist> velocitySubscriber = null;
//        Queue<List<Double>> cmdQueue;
//        private List<Double> vel_cmd;
        private double vl,vr;

        // control command subscriber
        Subscriber<std_msgs.String> ctlCmdSubscriber=null;
        String ctlCmd;
        AgentController agentController;

        //tf broadcaster
        TransformBroadcaster tfb = null;
        //frame
        String robotFrame;
        String odomFrame;
        String globalFrame;

        //for synchronization
        Time time = new Time();

        //pose array publisher, currently for test
        Publisher<PoseArray> poseArrayPublisher=null;
        PoseArray poseArray;
        int paSeq;
        Point3d previousPosition=null;

        public Robot(Vector3d position, double ori, int id) {
            // initialize position and orientation
            super(position, "robot"+id);

            this.id=id;
            this.radius = (float) Parameters.FOOTPRINT_RADIUS;// loaded from agent parameters
            orientation = ori;
            //use differential model
            kinematic = RobotFactory.setDifferentialDriveKinematicModel(this);
            wheelDistance = this.getRadius();
            // Add camera
            //camera = RobotFactory.addCameraSensor(this);
            laserScan = new LaserScan(this.radius,
                    SimParams.SCAN_ANGLE_RANGE / 180 * Math.PI,
                    SimParams.SCAN_SENSOR_NB,
                    SimParams.SCAN_MIN_RANGE,
                    SimParams.SCAN_MAX_RANGE,
                    SimParams.SCAN_UPDATE_FREQ);
            sensors = laserScan.getSensor();
            // if sensors are not on the center of the robot then height
            // should be assigned to the laserscan, in the robot frame
            this.addSensorDevice(sensors, new Vector3d(0, 0, 0), 0);
            //initialize frames
            robotFrame = this.getName() + "_base";
            odomFrame = this.getName() + "_odometry";
            globalFrame = "map";

            initPubSub();
            initController(
                    "robot" + id + "_rmq",
                    null,
                    "amqp://localhost:5672"
            );

        }

        private void initController(String name, Address[] addresses, String url) {
            agentController = new AgentController(name, addresses, url);
            NodeConfiguration configuration = NodeConfiguration.newPublic("localhost");
            agentController.start(configuration);
        }

        public void initPubSub() {
            // initialize node
            agentNode = new AgentNode(this.getName());
            node = agentNode.getNode();
            // publish robot numbers to setup planner numbers
            params=node.getParameterTree();
            params.set("robotNb",robotNb);
            params.set("posRadius",posRadius);
            // initialize odometry publisher and message
            if (odometryPublisher == null) {
                odometryPublisher = node.newPublisher(this.getName() + "/odometry", Odometry._TYPE);
                // initialize message
                odomMsg = odometryPublisher.newMessage();
                odomMsg.getHeader().setFrameId(odomFrame);
                odomMsg.setChildFrameId(robotFrame);
                odomSeq = 0;
            }
            //initialize laser scan publisher
            if (laserscanPublisher == null) {
                laserscanPublisher = node.newPublisher(this.getName() + "/scan/point_cloud2", PointCloud2._TYPE);
                // initialize message
                pctmp=laserscanPublisher.newMessage();
                pc2 = laserscanPublisher.newMessage();
                pc2Seq = 0;
            }

            //for test
            if (poseArrayPublisher==null){
                poseArrayPublisher=node.newPublisher(this.getName()+"/particlecloud",PoseArray._TYPE);
                poseArray=node.getTopicMessageFactory().newFromType(PoseArray._TYPE);
                poseArray.getHeader().setFrameId(robotFrame);
                paSeq=0;
            }

            //initialize velocity command subscriber
            if (velocitySubscriber == null) {
                velocitySubscriber = node.newSubscriber(this.getName() + "/cmd_vel", Twist._TYPE);
                velocitySubscriber.addMessageListener(new MessageListener<Twist>() {
                    @Override
                    public void onNewMessage(Twist msg) {
                        double v, w;
                        //in ros coordinate
                        Vector3d vel = new Vector3d(msg.getLinear().getX(), msg.getLinear().getY(), msg.getLinear().getZ());
                        //no need to transform the coordinate
                        v = vel.length();
                        w = msg.getAngular().getZ();
                        vl =  v - w * wheelDistance / 2;
                        vr =  v + w * wheelDistance / 2;
                    }
                });
            }

            if (startGoalPublisher == null) {
                startGoalPublisher = node.newPublisher(this.getName() + "/start_goal", PoseStamped._TYPE);
                startGoalMsg = startGoalPublisher.newMessage();
                startGoalMsg.getHeader().setFrameId(globalFrame);
                start = utilsSim.messageFactory.newFromType(Pose._TYPE);
                goal = utilsSim.messageFactory.newFromType(Pose._TYPE);
                sgseq = 0;
            }

            if (ctlCmdSubscriber==null){
                ctlCmdSubscriber=node.newSubscriber("/ctl_cmd", std_msgs.String._TYPE);
                ctlCmd = "";
                ctlCmdSubscriber.addMessageListener(new MessageListener<std_msgs.String>() {
                    @Override
                    public void onNewMessage(std_msgs.String string) {
                        ctlCmd=string.getData();
                    }
                });
            }

            if (tfb == null) {
                tfb = new TransformBroadcaster(node);
            }
        }

        /**
         * This method is called by the simulator engine on reset.
         */
        public void initBehavior() {
            float colorvalue=(float)this.id/robotNb;
            setColor(new Color3f(0,colorvalue,0));
            this.resetPosition();
            this.rotateY(orientation);
            vl=0;
            vr=0;
            sgseq = 0;
            agentController.clearQueues();
        }

        /**
         * This method is call cyclically (20 times per second)  by the simulator engine.
         */
        public void performBehavior() {
            // send out goal
            if (ctlCmd.equals("pause") || sgseq < 10)
                kinematic.setWheelsVelocity(0,0);
            else
                kinematic.setWheelsVelocity(vl,vr);

            time = node.getCurrentTime();
            //send transform
            sendTransform();
            //publish scan in frequency of 10Hz
            if (getCounter() % 1== 0 ) {
                pc2.getHeader().setSeq(pc2Seq++);
                pc2.getHeader().setStamp(time);
                //publish valid laser scan in pointcloud2 format in global frame
                laserScan.getLaserscanPointCloud2(pc2, this);
                laserscanPublisher.publish(pc2);
            }
            //test publish localization pose array
            if (getCounter()%2==0){
                Point3d cor = new Point3d();
                this.getCoords(cor);
                poseArray.getHeader().setStamp(time);
                poseArray.getHeader().setSeq(paSeq++);
                if (previousPosition==null||!cor.equals(previousPosition)){
                    if (previousPosition==null)
                        previousPosition=new Point3d();
                    this.getCoords(previousPosition);
                    setPoseArrayMsg();
                }
                poseArrayPublisher.publish(poseArray);
            }
            //send odometry in frequency of 10 HZ
            if (getCounter() % 1 == 0) {
                setOdomMsg(time);
                odometryPublisher.publish(odomMsg);
            }

            if (sgseq < 10) {

                getStartGoal();
                startGoalMsg.getHeader().setStamp(time);
                startGoalMsg.getHeader().setSeq(sgseq);

                // set start in even sequence
                if (sgseq % 2 == 0)
                    startGoalMsg.setPose(start);
                else
                    startGoalMsg.setPose(goal);
                startGoalPublisher.publish(startGoalMsg);
                sgseq++;
            }

        }

        private void getStartGoal() {
            Point3d start_cor = new Point3d();
            this.getCoords(start_cor);
            Point3d goal_cor = new Point3d();
            goal_cor.set(-start_cor.getX(), start_cor.getY(), -start_cor.getZ());

            Quat4d oriStart = getOrientation();
            Quat4d oriGoal = new Quat4d();
            Transform3D tfr = new Transform3D(oriStart, new Vector3d(0, 0, 0), 1);
            Transform3D tfrPI = new Transform3D(new Quat4d(0, 1, 0, 0), new Vector3d(), 1);
            tfr.mul(tfrPI);
            tfr.get(oriGoal);

            // transform coordinates
            start = utilsSim.getPose(start_cor, oriStart);
            goal = utilsSim.getPose(goal_cor, oriGoal);
        }

        public void setOdomMsg(Time t) {
            //get position and velocities, all in odometry frame
            Point3d cor = new Point3d();
            this.getCoords(cor);
            Quat4d ori = getOrientation();

            odomMsg.getHeader().setStamp(t);
            odomMsg.getHeader().setSeq(odomSeq++);

            odomMsg.getPose().setPose(utilsSim.getPose(cor, ori));
            odomMsg.getTwist().getTwist().setLinear(utilsSim.getTwist(this.linearVelocity));
            odomMsg.getTwist().getTwist().setAngular(utilsSim.getTwist(this.angularVelocity));

        }

        public Quat4d getOrientation() {
            //get orientation
            Transform3D tfr = new Transform3D();
            Quat4d ori = new Quat4d();
            this.getRotationTransform(tfr);
            tfr.get(ori);
            return ori;
        }

        public void sendTransform() {
            String childFrame = robotFrame;
            String parentFrame = odomFrame;

            Transform3D tf;
            Vector3d tft3d = new Vector3d();
            Quat4d tfrq = new Quat4d();

            tf=getTransform();
            tf.get(tft3d);
            tf.get(tfrq);

            utilsSim.toROSCoordinate(tft3d);
            utilsSim.toROSCoordinate(tfrq);

            tfb.sendTransform(
                    parentFrame, childFrame,
                    tft3d.getX(), tft3d.getY(), tft3d.getZ(),
                    tfrq.getX(), tfrq.getY(), tfrq.getZ(), tfrq.getW()
            );
            // currently map frame and odometry frame are the same
            childFrame = odomFrame;
            parentFrame = globalFrame;
            tfb.sendTransform(
                    parentFrame, childFrame,
                    0, 0, 0,
                    0, 0, 0, 1
            );
        }

        public Transform3D getTransform(){
            Transform3D tf = new Transform3D();
            Vector3d tft3d = new Vector3d();
            Quat4d tfrq = new Quat4d();
            this.getTranslationTransform(tf);
            tf.get(tft3d);
            this.getRotationTransform(tf);
            tf.get(tfrq);
            tf.set(tfrq, tft3d, 1);
            return tf;
        }

        public void setPoseArrayMsg(){
            List<Pose> pa=new ArrayList<Pose>();
            //get orientation
            Transform3D tfr = new Transform3D();
            this.getRotationTransform(tfr);

            Quat4d ori = new Quat4d();
            Point3d pt= new Point3d();
            for (int i = 0; i <50 ; i++) {
                // in robot base frame
                pt.setX(utilsSim.getGaussianNoise(0, this.radius/10));
                pt.setZ(utilsSim.getGaussianNoise(0, this.radius / 10));
                tfr.rotY(utilsSim.getGaussianNoise(0, 0.15));
                tfr.get(ori);
                pa.add(utilsSim.getPose(pt, ori));
            }
            poseArray.setPoses(pa);
        }

        public void shutDown() {
            agentController.stop();
            agentNode.shutDown();
        }

    }



    /**
     * Describe the environement
     */
    static public class MyEnv extends EnvironmentDescription {
        public MyEnv() {

            light1IsOn = true;
            light2IsOn = false;

            Wall w1 = new Wall(new Vector3d(9, 0, 0), 19, 2, this);
            w1.rotate90(1);
            add(w1);
            Wall w2 = new Wall(new Vector3d(-9, 0, 0), 19, 2, this);
            w2.rotate90(1);
            add(w2);
            Wall w3 = new Wall(new Vector3d(0, 0, 9), 19, 2, this);
            add(w3);
            Wall w4 = new Wall(new Vector3d(0, 0, -9), 19, 2, this);
            add(w4);

// there are bugs in avoid obstacles
//            Box b1 = new Box(new Vector3d(0, 0, 0), new Vector3f((float)0.2, 1, 3),this);
//            add(b1);
//            Vector3d pose1 = new Vector3d(posRadius * Math.cos(Math.PI/3), 0, -posRadius * Math.sin(Math.PI/3));
//            add(new Robot(pose1, Math.PI + Math.PI/3, "robot0"));

//            add(new Arch(new Vector3d(3, 0, -3), this));
            robots.clear();
            double step = 2 * Math.PI / robotNb;
            for (int i = 0; i < robotNb; i++) {
                Vector3d pose = new Vector3d(posRadius * Math.cos(i * step), 0, -posRadius * Math.sin(i * step));
                Robot robot = new Robot(pose, Math.PI + i * step, i);
                robots.add(robot);
                add(robot);
            }
        }
    }

    public static void main(String[] args) {
        // request antialising
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new MyEnv(), false);

        Runtime.getRuntime().addShutdownHook(new Thread() {
            public void run() {
                for (Robot robot : robots) {
                    System.out.println("Shutting down " + robot.getName());
                    robot.shutDown();
                }
            }
        });
    }

} 