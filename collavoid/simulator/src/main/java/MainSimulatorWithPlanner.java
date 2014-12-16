import cgl.iotrobots.collavoid.GlobalPlanner.GlobalPlanner;
import cgl.iotrobots.collavoid.LocalPlanner.LocalPlanner;
import cgl.iotrobots.collavoid.utils.Parameters;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.PoseStamped;
import geometry_msgs.Twist;
import nav_msgs.Odometry;
import org.apache.commons.logging.Log;
import org.ros.RosCore;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.pubsub.TransformBroadcaster;
import org.ros.rosjava.tf.pubsub.TransformListener;
import sensor_msgs.PointCloud2;
import simbad.gui.Simbad;
import simbad.sim.*;
import utils.SimParams;

import javax.media.j3d.Transform3D;
import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;

public class MainSimulatorWithPlanner {
    static final int robotNb = SimParams.ROBOT_NB;
    static final double posRadius = SimParams.POSE_RADIUS;
    /**
     * Describe the robot
     */
    static public class Robot extends Agent {

        // id
        int id;

        //orientation
        double orientation;
        //kinematic
        DifferentialKinematic kinematic;
        private double vl;
        private double vr;
        private double wheelDistance;
        //sensors
        LaserScan laserScan;
        RangeSensorBelt sensors;
        //CameraSensor camera;
        //node
        private ConnectedNode node;

        //odometry publisher
        Publisher<Odometry> odometryPublisher = null;
        Odometry odomMsg;
        int odomSeq;
        //laser scan publisher
        Publisher<PointCloud2> laserscanPublisher = null;
        PointCloud2 pc2,pctmp;
        int pc2Seq;
        //velocity command publisher
        Publisher<Twist> velocityPublisher = null;
        Twist cmd_vel;

        //velocity command subscriber
        Subscriber<Twist> velocitySubscriber = null;
        //tf listener
        TransformListener tfl = null;
        //tf broadcaster
        TransformBroadcaster tfb = null;
        //frame
        String robotFrame;
        String odomFrame;
        String globalFrame;
        //planner
        GlobalPlanner globalPlanner = null;
        LocalPlanner localPlanner = null;
        List<PoseStamped> globalPlan = null;
        Point3d start;
        Point3d goal;
        Quat4d oriGoal,oriStart;
        boolean plannerSet;
        //for synchronization
        Time time = new Time();

        //pose array publisher, currently for test
        Publisher<PoseArray> poseArrayPublisher=null;
        PoseArray poseArray;
        int paSeq;
        Point3d previousPosition=null;


        public Robot(Vector3d position, double ori, int id) {
            super(position, "robot" + id);
            this.id = id;
            //set color
            float colorvalue = (float) this.id / robotNb;
            setColor(new Color3f(0, colorvalue, 0));

            // initialize position and orientation
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

        }

        public void initPubSub() {
            // initialize node
            AgentNode agentNode = new AgentNode(this.getName());
            node = agentNode.getNode();
            Log log=node.getLog();
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
                pc2.getHeader().setFrameId(robotFrame);
                pc2Seq = 0;
            }
            if (velocityPublisher == null) {
                velocityPublisher = node.newPublisher(this.getName() + "/cmd_vel", Twist._TYPE);
                cmd_vel = velocityPublisher.newMessage();
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
                },10);
            }
            if (tfl == null) {
                tfl = new TransformListener(node);
            }

            if (tfb == null) {
                tfb = new TransformBroadcaster(node);
            }
        }

        /**
         * This method is called by the simulator engine on reset.
         */
        public void initBehavior() {
            this.rotateY(orientation);
            initPlanner();
        }

        /**
         * This method is call cyclically (20 times per second)  by the simulator engine.
         */
        public void performBehavior() {

            kinematic.setWheelsVelocity(vl, vr);
            time = node.getCurrentTime();
            //send transform
            sendTransform();
            //publish scan in frequency of 10Hz
            if (getCounter() % 2== 0 ) {
                pc2.getHeader().setSeq(pc2Seq++);
                pc2.getHeader().setStamp(time);
                //publish valid laser scan in pointcloud2 format in global frame
                laserScan.getLaserscanPointCloud2(pc2,this);
                laserscanPublisher.publish(pc2);
            }
            //test, publish localization pose array
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

                // delay some time to set the planner
                if (!plannerSet && getCounter() % 20 == 0) {
                plannerSet=true;
                if (!localPlanner.setPlan(globalPlan))
                    node.getLog().error("Set global plan error!");
            }

                //control frequency is 20hz
                if (localPlanner.computeVelocityCommands(cmd_vel));
                velocityPublisher.publish(cmd_vel);
            }
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

            Transform3D tf = new Transform3D();
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
                pt.setX(utilsSim.getGaussianNoise(0, this.radius / 10));
                pt.setZ(utilsSim.getGaussianNoise(0, this.radius / 10));
                tfr.rotY(utilsSim.getGaussianNoise(0, 0.15));
                tfr.get(ori);
                pa.add(utilsSim.getPose(pt, ori));
            }
            poseArray.setPoses(pa);
        }

        private void initPlanner(){
            if (globalPlan == null) {
                globalPlan = new ArrayList<PoseStamped>();
            } else {
                globalPlan.clear();
            }

            if (globalPlanner == null)
                globalPlanner = new GlobalPlanner();

            if (localPlanner == null)
                localPlanner = new LocalPlanner(node, tfl);

            //set start point and goal
            start = new Point3d();
            this.getCoords(start);
            goal = new Point3d();
            goal.set(-start.getX(), start.getY(), -start.getZ());

            oriStart = getOrientation();
            oriGoal=new Quat4d();
            Transform3D tfr = new Transform3D(oriStart, new Vector3d(0, 0, 0), 1);
            Transform3D tfrPI=new Transform3D(new Quat4d(0,1,0,0),new Vector3d(),1);
            tfr.mul(tfrPI);
            tfr.get(oriGoal);

            // transform coordinates
            start = utilsSim.toROSCoordinate(start);
            goal = utilsSim.toROSCoordinate(goal);
            utilsSim.toROSCoordinate(oriGoal);
            utilsSim.toROSCoordinate(oriStart);

            PoseStamped startPose = node.getTopicMessageFactory().newFromType(PoseStamped._TYPE);
            PoseStamped goalPose = node.getTopicMessageFactory().newFromType(PoseStamped._TYPE);

            startPose.getHeader().setFrameId(globalFrame);
            startPose.getPose().getPosition().setX(start.getX());
            startPose.getPose().getPosition().setY(start.getY());
            startPose.getPose().getOrientation().setX(oriStart.getX());
            startPose.getPose().getOrientation().setY(oriStart.getY());
            startPose.getPose().getOrientation().setZ(oriStart.getZ());
            startPose.getPose().getOrientation().setW(oriStart.getW());

            goalPose.getHeader().setFrameId(globalFrame);
            goalPose.getPose().getPosition().setX(goal.getX());
            goalPose.getPose().getPosition().setY(goal.getY());
            goalPose.getPose().getOrientation().setX(oriGoal.getX());
            goalPose.getPose().getOrientation().setY(oriGoal.getY());
            goalPose.getPose().getOrientation().setZ(oriGoal.getZ());
            goalPose.getPose().getOrientation().setW(oriGoal.getW());

            globalPlanner.makePlan(startPose, goalPose, globalPlan);

            plannerSet=false;
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

//            Box b1 = new Box(new Vector3d(0, 0, 0), new Vector3f((float)0.2, 1, 3),this);
//            add(b1);
//            Vector3d pose1 = new Vector3d(posRadius * Math.cos(Math.PI/3), 0, -posRadius * Math.sin(Math.PI/3));
//            add(new Robot(pose1, Math.PI + Math.PI/3, "robot0"));

//            add(new Arch(new Vector3d(3, 0, -3), this));
            double step = 2 * Math.PI / robotNb;
            for (int i = 0; i < robotNb; i++) {
                Vector3d pose = new Vector3d(posRadius * Math.cos(i * step), 0, -posRadius * Math.sin(i * step));
                add(new Robot(pose, Math.PI + i * step, i));
            }
        }
    }

    public static void main(String[] args) {
        doShutDownWork();
        // run roscore
        RosConnect(); // not working
        // request antialising
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new MyEnv(), false);

    }

    static RosCore rosCore;

    public static void RosConnect() {
        rosCore = RosCore.newPublic();
        rosCore.start();

        try {
            rosCore.awaitStart();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    private static void doShutDownWork() {
        Runtime.getRuntime().addShutdownHook(new Thread() {
            public void run() {
                rosCore.shutdown();
            }
        });
    }
} 