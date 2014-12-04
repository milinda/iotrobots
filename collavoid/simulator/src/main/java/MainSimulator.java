import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.Twist;
import nav_msgs.Odometry;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.Transform;
import org.ros.rosjava.tf.pubsub.TransformBroadcaster;
import org.ros.rosjava.tf.pubsub.TransformListener;
import sensor_msgs.PointCloud2;
import simbad.gui.Simbad;
import simbad.sim.*;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import java.util.ArrayList;
import java.util.List;


public class MainSimulator {

    /**
     * Describe the robot
     */
    static public class Robot extends Agent {

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
        CameraSensor camera;
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
        InitPlannerThread plannerThread;
        //for synchronization
        Time time = new Time();

        //pose array publisher, currently for test
        Publisher<PoseArray> poseArrayPublisher=null;
        PoseArray poseArray;
        int paSeq;
        Point3d previousPosition=null;


        public Robot(Vector3d position, double ori, String name) {
            // initialize position and orientation
            super(position, name);
            orientation = ori;
            //use differential model
            kinematic = RobotFactory.setDifferentialDriveKinematicModel(this);
            wheelDistance = this.getRadius();
            // Add camera
            camera = RobotFactory.addCameraSensor(this);
            laserScan = new LaserScan(this.getRadius(), Math.PI, 180, 1.5, 3);
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
                        Vector3d vel = new Vector3d(msg.getLinear().getX(), msg.getLinear().getY(), msg.getLinear().getZ());
                        v = vel.length();
                        w = msg.getAngular().getZ();
                        vl = (2 * v - w * wheelDistance) / 2;
                        vr = (2 * v + w * wheelDistance) / 2;
                    }
                });
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

            //set start point and goal
            Point3d start = new Point3d();
            this.getCoords(start);
            Point3d goal = new Point3d();
            goal.set(-start.getX(), start.getY(), -start.getZ());

            Quat4d oriGoal = getOrientation();
            Transform3D tfr = new Transform3D(oriGoal, new Vector3d(0, 0, 0), 1);
            tfr.rotY(Math.PI);
            tfr.get(oriGoal);

            plannerThread = new InitPlannerThread(node, tfl, start, goal, oriGoal);
        }

        /**
         * This method is call cyclically (20 times per second)  by the simulator engine.
         */
        public void performBehavior() {

            kinematic.setWheelsVelocity(vl, vr);

            time = node.getCurrentTime();
            //send transform
            sendTransform(time);
            //publish scan in frequency of 1Hz
            if (getCounter() % 20 == 0 ) {
                pc2.getHeader().setSeq(pc2Seq++);
                pc2.getHeader().setStamp(time);
                //publish valid laser scan in pointcloud2 format

                if(laserScan.getLaserscanPointCloud2(pctmp))
                    laserScan.getLaserscanPointCloud2(pc2);
                laserscanPublisher.publish(pc2);
            }
            //test
            if (getCounter()%20==0){
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
            if (getCounter() % 2 == 0) {
                setOdomMsg(time);
                odometryPublisher.publish(odomMsg);

                //control frequency is 10hz
                //localPlanner.computeVelocityCommands(cmd_vel);
                //velocityPublisher.publish(cmd_vel);
            }
        }

        public void setOdomMsg(Time t) {
            //get position and velocities, all in odometry frame
            Point3d cor = new Point3d();
            this.getCoords(cor);
            //cor.scale(10);
            Quat4d ori = getOrientation();

            odomMsg.getHeader().setStamp(t);
            odomMsg.getHeader().setSeq(odomSeq++);

            odomMsg.getPose().setPose(utils.getPose(cor, ori));
            odomMsg.getTwist().getTwist().setLinear(utils.getTwist(this.linearVelocity));
            odomMsg.getTwist().getTwist().setAngular(utils.getTwist(this.angularVelocity));

        }

        public Quat4d getOrientation() {
            //get orientation
            Transform3D tfr = new Transform3D();
            Quat4d ori = new Quat4d();
            this.getRotationTransform(tfr);
            tfr.get(ori);
            return ori;
        }

        public void sendTransform(Time t) {
            String childFrame = robotFrame;
            String parentFrame = odomFrame;

            Transform3D tf = new Transform3D();
            Vector3d tft3d = new Vector3d();
            Quat4d tfrq = new Quat4d();

            this.getTranslationTransform(tf);
            tf.get(tft3d);
            this.getRotationTransform(tf);
            tf.get(tfrq);
            tf.set(tfrq,tft3d,1);

            tf.invert();

            tf.get(tft3d);
            tf.get(tfrq);

            utils.toROSCoordinate(tft3d);
            utils.toROSCoordinate(tfrq);

            tfb.sendTransform(
                    parentFrame, childFrame,
                    t,
                    tft3d.getX(), tft3d.getY(), tft3d.getZ(),
                    tfrq.getX(), tfrq.getY(), tfrq.getZ(), tfrq.getW()
            );
            // currently map frame and odometry frame are the same
            childFrame = odomFrame;
            parentFrame = globalFrame;
            tfb.sendTransform(
                    parentFrame, childFrame,
                    t,
                    0, 0, 0,
                    0, 0, 0, 1
            );
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
                pt.setX(utils.getGaussianNoise(0, this.radius/10));
                pt.setZ(utils.getGaussianNoise(0, this.radius / 10));
                tfr.rotY(utils.getGaussianNoise(0, 0.15));
                tfr.get(ori);
                pa.add(utils.getPose(pt, ori));
            }
            poseArray.setPoses(pa);
        }
    }

    /**
     * Describe the environement
     */
    static public class MyEnv extends EnvironmentDescription {
        public MyEnv() {
            final int robotNb = 5;
            final double posRadius = 6;

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

            Box b1 = new Box(new Vector3d(4, 0, 0), new Vector3f(1, 1, 1),
                    this);
            add(b1);
//            add(new Arch(new Vector3d(3, 0, -3), this));
            double step = 2 * Math.PI / robotNb;
            for (int i = 0; i < robotNb; i++) {
                Vector3d pose = new Vector3d(posRadius * Math.cos(i * step), 0, -posRadius * Math.sin(i * step));
                add(new Robot(pose, Math.PI + i * step, "robot" + i));
            }
        }
    }

    public static void main(String[] args) {
        // request antialising
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new MyEnv(), false);

    }

} 