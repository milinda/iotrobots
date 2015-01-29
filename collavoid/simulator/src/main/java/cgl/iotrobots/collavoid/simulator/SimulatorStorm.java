package cgl.iotrobots.collavoid.simulator;

import cgl.iotrobots.collavoid.commons.planners.Parameters;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.controller.storm.AgentControllerStorm;
import com.rabbitmq.client.Address;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.PoseStamped;
import geometry_msgs.Twist;
import nav_msgs.Odometry;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import sensor_msgs.PointCloud2;
import simbad.gui.Simbad;
import simbad.sim.*;

import javax.media.j3d.Transform3D;
import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;

public class SimulatorStorm {
    static final int robotNb = Parameters.ROBOT_NUMBER;
    static final double posRadius = Parameters.POSE_RADIUS;
    static List<Robot> robots = new ArrayList<Robot>();

    //test
    static int reachGoalNo;
    static int startedNo;
    //test

    public static class WheelVelocity {
        private double vl;
        private double vr;
        private boolean isNew;

        public void setVl(double vl) {
            this.vl = vl;
        }

        public void setVr(double vr) {
            this.vr = vr;
        }

        public void setIsNew(boolean state) {
            isNew = state;
        }

        public double getVl() {
            return vl;
        }

        public double getVr() {
            return vr;
        }

        public void reset() {
            vl = 0;
            vr = 0;
        }

        public boolean getIsNew() {
            return isNew;
        }

    }

    /**
     * Describe the robot
     */
    static public class Robot extends Agent {

        //test
        long lastnewvel;
        boolean startAgain;
        //test

        //id
        int id;
        String robotName;

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
        RangeSensorBelt bumpers;

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
        PointCloud2 pc2, pctmp;
        int pc2Seq;

        //velocity command subscriber
        Subscriber<Twist> velocitySubscriber = null;
        WheelVelocity wheelVelocity = new WheelVelocity();

        private double vl, vr;

        // control command subscriber
        Subscriber<std_msgs.String> ctlCmdSubscriber = null;
        String ctlCmd;
        AgentControllerStorm agentController;

        //frame
        String robotFrame;
        String odomFrame;
        String globalFrame;

        //for synchronization
        Time time = new Time();

        //pose array publisher, currently for test
        Publisher<PoseArray> poseArrayPublisher = null;
        PoseArray poseArray;
        int paSeq;
        Point3d previousPosition = null;

        public Robot(Vector3d position, double ori, int id) {
            // initialize position and orientation
            super(position, "robot" + id);
            this.id = id;
            this.robotName = "robot" + id;
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
            bumpers = RobotFactory.addBumperBeltSensor(this, 12);
            // if sensors are not on the center of the robot then height
            // should be assigned to the laserscan, in the robot frame
            this.addSensorDevice(sensors, new Vector3d(0, 0, 0), 0);
            //initialize frames
            robotFrame = this.getName() + "_base";
            odomFrame = this.getName() + "_odometry";
            globalFrame = "map";

            initPubSub();
            initController(
                    "robot" + id,
                    null,
                    Constant_msg.RMQ_URL
            );

        }

        private void initController(String name, Address[] addresses, String url) {
            agentController = new AgentControllerStorm(name, addresses, url);
            NodeConfiguration configuration = NodeConfiguration.newPublic("localhost");
            agentController.start(configuration);
        }

        public void initPubSub() {
            // initialize node
            agentNode = new AgentNode(this.getName());
            node = agentNode.getNode();
            // publish robot numbers to setup planner numbers
            params = node.getParameterTree();
            params.set("robotNb", robotNb);
            params.set("posRadius", posRadius);
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
                pctmp = laserscanPublisher.newMessage();
                pc2 = laserscanPublisher.newMessage();
                pc2.getHeader().setFrameId(globalFrame);
                pc2Seq = 0;
            }

            //for test
            if (poseArrayPublisher == null) {
                poseArrayPublisher = node.newPublisher(this.getName() + "/particlecloud", PoseArray._TYPE);
                poseArray = node.getTopicMessageFactory().newFromType(PoseArray._TYPE);
                poseArray.getHeader().setFrameId(robotFrame);
                paSeq = 0;
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

                        vl = v - w * wheelDistance / 2;
                        vr = v + w * wheelDistance / 2;
                        lastnewvel = System.currentTimeMillis();

                        //test
//                       if (id==0)
//                      System.out.println("robot"+id+" New cmd delay: "+(System.currentTimeMillis()-lastnewvel));
//                        wheelVelocity.setVl(v - w * wheelDistance / 2);
//                        wheelVelocity.setVr(v + w * wheelDistance / 2);
//                        wheelVelocity.setIsNew(true);
                        if (agentController.isGoalReached()) {
                            reachGoalNo++;
                            startAgain = true;
                            startedNo = 0;
                        }
                        //test

                    }
                }, 5);
            }

            if (startGoalPublisher == null) {
                startGoalPublisher = node.newPublisher(this.getName() + "/start_goal", PoseStamped._TYPE);
                startGoalMsg = startGoalPublisher.newMessage();
                startGoalMsg.getHeader().setFrameId(globalFrame);
                start = utilsSim.messageFactory.newFromType(Pose._TYPE);
                goal = utilsSim.messageFactory.newFromType(Pose._TYPE);
                sgseq = 0;
            }

            if (ctlCmdSubscriber == null) {
                ctlCmdSubscriber = node.newSubscriber("/ctl_cmd", std_msgs.String._TYPE);
                ctlCmd = "";
                ctlCmdSubscriber.addMessageListener(new MessageListener<std_msgs.String>() {
                    @Override
                    public void onNewMessage(std_msgs.String string) {
                        ctlCmd = string.getData();
                    }
                });
            }
        }

        /**
         * This method is called by the simulator engine on startAgain.
         */
        public void initBehavior() {
            agentController.clearQueues();
            this.resetPosition();
            this.rotateY(orientation);
            sgseq = 0;
            startAgain = false;
            wheelVelocity.reset();
            vl = 0;
            vr = 0;
            float colorvalue = (float) this.id / robotNb;
            setColor(new Color3f(0, colorvalue, 0));
        }

        /**
         * This method is call cyclically (20 times per second)  by the simulator engine.
         */
        public void performBehavior() {
            // send out goal
            if (ctlCmd.equals("pause") || sgseq < 10) {
                kinematic.setWheelsVelocity(0, 0);
            } else {
                kinematic.setWheelsVelocity(vl, vr);
//                setKinematic(wheelVelocity);
            }

            time = node.getCurrentTime();

            //publish scan in frequency of 10Hz
            if (getCounter() % 1 == 0) {
                pc2.getHeader().setSeq(pc2Seq++);
                pc2.getHeader().setStamp(time);
                //publish valid laser scan in pointcloud2 format in global frame
                laserScan.getLaserscanPointCloud2(pc2, this.getTransform());
                laserscanPublisher.publish(pc2);
            }
            //test publish localization pose array
            if (getCounter() % 2 == 0) {
                Point3d cor = new Point3d();
                this.getCoords(cor);
                poseArray.getHeader().setStamp(time);
                poseArray.getHeader().setSeq(paSeq++);
                if (previousPosition == null || !cor.equals(previousPosition)) {
                    if (previousPosition == null)
                        previousPosition = new Point3d();
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

            //test start again when all reached goal
//            if (reachGoalNo >= robotNb && startAgain) {
//                sgseq = 0;
//                this.startAgain = false;
//                if (++startedNo >= robotNb)
//                    reachGoalNo = 0;
//            }
            //test start again once me reached goal
            if (startAgain) {
                sgseq = 0;
                this.startAgain = false;
            }
            if (bumpers.oneHasHit()) {
                System.out.println("Collision detected: " + robotName);
            }
            //test

        }

        private void setKinematic(WheelVelocity wv) {
            double vln, vrn;
            if (wheelVelocity.getIsNew()) {

//                vln=Math.max(wv.getVl(), (vl +Math.signum(wv.getVl()-vl)*
//                        Parameters.ACC_LIM_X / Parameters.CONTROLLER_FREQUENCY));
//                vrn=Math.max(wv.getVr(), (vr +Math.signum(wv.getVr()-vr)*
//                        Parameters.ACC_LIM_X / Parameters.CONTROLLER_FREQUENCY));
                vln = setv(vl, wv.getVl());
                vrn = setv(vr, wv.getVr());
                kinematic.setWheelsVelocity(vln, vrn);
                wheelVelocity.setIsNew(false);
            } else {
//                vln=Math.max(0.0, (vl +Math.signum(0-vl)*
//                        Parameters.ACC_LIM_X / Parameters.CONTROLLER_FREQUENCY));
//                vrn=Math.max(0.0, (vr +Math.signum(0-vr)*
//                        Parameters.ACC_LIM_X / Parameters.CONTROLLER_FREQUENCY));
                vln = setv(vl, vl);
                vrn = setv(vr, vr);
                kinematic.setWheelsVelocity(vln, vrn);
            }
            vl = vln;
            vr = vrn;
        }

        private double setv(double a, double b) {
            double sign = Math.signum(b - a);
            return sign * Math.min(b * sign, sign * (a + sign * Parameters.ACC_LIM_X / Parameters.CONTROLLER_FREQUENCY / 5));
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


        public Transform3D getTransform() {
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

        public void setPoseArrayMsg() {
            List<Pose> pa = new ArrayList<Pose>();
            //get orientation
            Transform3D tfr = new Transform3D();
            this.getRotationTransform(tfr);

            Quat4d ori = new Quat4d();
            Point3d pt = new Point3d();
            for (int i = 0; i < 50; i++) {
                // in robot base frame
                pt.setX(utilsSim.getGaussianNoise(0, this.radius / 10));
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
