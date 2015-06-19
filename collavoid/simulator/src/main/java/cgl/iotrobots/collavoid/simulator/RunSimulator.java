package cgl.iotrobots.collavoid.simulator;

import cgl.iotrobots.collavoid.commons.planners.Parameters;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.Twist;
import nav_msgs.Odometry;
import org.apache.commons.cli.*;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import sensor_msgs.PointCloud2;
import simbad.gui.Simbad;
import simbad.sim.*;

import javax.media.j3d.Transform3D;
import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RunSimulator {
    public static Logger LOG = LoggerFactory.getLogger(RunSimulator.class);

    static int robotNb = Parameters.ROBOT_NUMBER;
    static double posRadius = Parameters.POSE_RADIUS;
    static String ROS_IP = "localhost";
    static String ROS_MASTER_RUI = "http://localhost:11311";
    static List<Robot> robots = new ArrayList<Robot>();

    private static final String ROBOT_NUMBER_ARG_NAME = "n";
    private static final String ROBOT_NUMBER_KEY = "robotNumber";
    private static final String ROBOT_POSE_RADIUS_ARG_NAME = "r";
    private static final String ROBOT_POSE_RADIUS_KEY = "robotPoseRadius";
    private static final String ROS_IP_ARG_NAME = "ip";
    private static final String ROS_IP_KEY = "rosIp";
    private static final String ROS_MASTER_RUI_ARG_NAME = "master";
    private static final String ROS_MASTER_RUI_KEY = "masterRui";

    /**
     * Describe the robot
     */
    static public class Robot extends Agent {
        //idx
        int idx;
        String robotName;
        //for shutting down
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
        //odometry publisher
        Publisher<Odometry> odometryPublisher = null;
        Odometry odomMsg;
        int odomSeq;
        //laser scan publisher
        Publisher<PointCloud2> laserscanPublisher = null;
        PointCloud2 pc2, pctmp;
        int pc2Seq;
        //pose array publisher, currently for test
        Publisher<PoseArray> poseArrayPublisher = null;
        PoseArray poseArray;
        int paSeq;
        Point3d previousPosition = null;
        //velocity command subscriber
        Subscriber<Twist> velocitySubscriber = null;
        private double vl, vr;
        // control command pub/sub
        Publisher<std_msgs.String> ctlCmdPublisher = null;
        std_msgs.String ctlCmd;
        //frame
        String robotFrame;
        String odomFrame;
        String globalFrame;
        //for time synchronization
        Time time = new Time();
        // to show collision
        LampActuator lamp;
        int lamponcnt;

        public Robot(Vector3d position, double ori, int id) {
            // initialize position and orientation
            super(position, "robot" + id);
            this.idx = id;
            this.robotName = "robot" + id;
            this.radius = (float) (Parameters.FOOTPRINT_RADIUS);// loaded from agent parameters
            orientation = ori;
            lamp = RobotFactory.addLamp(this);
            // use differential model
            kinematic = RobotFactory.setDifferentialDriveKinematicModel(this);
            wheelDistance = this.getRadius();
            // add sensors
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
            //initialize frames, currently frame is not used
            robotFrame = this.getName() + "_base";
            odomFrame = this.getName() + "_odometry";
            globalFrame = "map";

            initPubSub();
        }

        public void initPubSub() {
            // initialize node
            agentNode = new AgentNode(this.getName(), ROS_IP, ROS_MASTER_RUI);
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
                pctmp = laserscanPublisher.newMessage();
                pc2 = laserscanPublisher.newMessage();
                pc2.getHeader().setFrameId(globalFrame);
                pc2Seq = 0;
            }

            //initialize localization publisher, for test
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
                    }
                }, 1);
            }

            if (ctlCmdPublisher == null) {
                ctlCmdPublisher = node.newPublisher(this.getName() + "/ctl_cmd", std_msgs.String._TYPE);
                ctlCmd = ctlCmdPublisher.newMessage();
            }
        }

        /**
         * This method is called by the simulator engine on startAgain.
         */
        public void initBehavior() {
            this.resetPosition();
            this.rotateY(orientation);
            vl = 0;
            vr = 0;
            lamp.setOn(false);
            ctlCmd.setData(Constant_storm.Command.RESET_CMD);
            LOG.info("{} send out reset cmd", this.getName());
            ctlCmdPublisher.publish(ctlCmd);
            setRobotColor();
        }

        private void setRobotColor() {
            if (this.idx % 3 == 0) {
                float colorvalue = (float) this.idx / robotNb;
                setColor(new Color3f(colorvalue, 0, 0));
            } else if (this.idx % 3 == 1) {
                float colorvalue = (float) this.idx / robotNb;
                setColor(new Color3f(0, colorvalue, 0));
            } else {
                float colorvalue = (float) this.idx / robotNb;
                setColor(new Color3f(0, 0, colorvalue));
            }
        }

        /**
         * This method is call cyclically (20 times per second)  by the simulator engine.
         */
        public void performBehavior() {
            kinematic.setWheelsVelocity(vl, vr);
//
            // use java system time to make sure time are synchronized
            time = Time.fromMillis(System.currentTimeMillis());

            // publish scan in frequency of 20Hz
            if (getCounter() % 1 == 0) {
                pc2.getHeader().setSeq(pc2Seq++);
                pc2.getHeader().setStamp(time);
                //publish valid laser scan in pointcloud2 format in global frame
                laserScan.getLaserscanPointCloud2(pc2, this.getTransform());
                laserscanPublisher.publish(pc2);
            }

            // test, publish localization pose array in 10 Hz
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

            // publish odometry in frequency of 20 Hz
            if (getCounter() % 1 == 0) {
                setOdomMsg(time);
                odometryPublisher.publish(odomMsg);
            }
            checkHit();
        }

        private void checkHit() {
            if (this.anOtherAgentIsVeryNear()) {
                lamp.setOn(true);
                lamponcnt = 30;
            }
            if (lamp.getOn() && lamponcnt >= 0) {
                lamponcnt--;
            }
            if (lamponcnt < 0)
                lamp.setOn(false);
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

        //      simulated pose array
        public void setPoseArrayMsg() {
            List<Pose> pa = new ArrayList<Pose>();
            //get orientation
            Transform3D tfr = new Transform3D();
            this.getRotationTransform(tfr);

            Quat4d ori = new Quat4d();
            Point3d pt = new Point3d();
            for (int i = 0; i < 50; i++) {
                // in robot base frame
                pt.setX(utilsSim.getGaussianNoise(0, this.radius * 2));
                pt.setZ(utilsSim.getGaussianNoise(0, this.radius * 2));
                tfr.rotY(utilsSim.getGaussianNoise(0, 0.5));
                tfr.get(ori);
                pa.add(utilsSim.getPose(pt, ori));
            }
            poseArray.setPoses(pa);
        }

        public void shutdown() {
            agentNode.shutdown();
        }

    }

    /**
     * Describe the environement
     */
    static public class MyEnv extends EnvironmentDescription {

        public MyEnv() {
            light1IsOn = true;
            light2IsOn = false;
            setRobotsPoseCircle();
        }

        public void setRobotsPoseCircle() {
            float worldsize = (float) posRadius * 2 + 4;
            setWorldSize(worldsize);

            Wall w1 = new Wall(new Vector3d(worldsize / 2, 0, 0), worldsize, 2, this);
            w1.rotate90(1);
            add(w1);
            Wall w2 = new Wall(new Vector3d(-worldsize / 2, 0, 0), worldsize, 2, this);
            w2.rotate90(1);
            add(w2);
            Wall w3 = new Wall(new Vector3d(0, 0, worldsize / 2), worldsize, 2, this);
            add(w3);
            Wall w4 = new Wall(new Vector3d(0, 0, -worldsize / 2), worldsize, 2, this);
            add(w4);

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

        Map<String, String> params = getProperties(args);
        if (params != null) {
            if (params.get(ROBOT_NUMBER_KEY) != null)
                robotNb = Integer.parseInt(params.get(ROBOT_NUMBER_KEY));
            if (params.get(ROBOT_POSE_RADIUS_KEY) != null)
                posRadius = Double.parseDouble(params.get(ROBOT_POSE_RADIUS_KEY));
            if (params.get(ROS_IP_KEY) != null)
                ROS_IP = params.get(ROS_IP_KEY);
            if (params.get(ROS_MASTER_RUI_KEY) != null)
                ROS_MASTER_RUI = params.get(ROS_MASTER_RUI_KEY);
        }

        // create Simbad instance with given environment, can not use background mode
        Simbad frame = new Simbad(new MyEnv(), false);

        Runtime.getRuntime().addShutdownHook(new Thread() {
            public void run() {
                for (Robot robot : robots) {
                    System.out.println("Shutting down " + robot.getName());
                    robot.shutdown();
                }
            }
        });
    }

    private static Map<String, String> getProperties(String[] args) {
        Map<String, String> conf = new HashMap<String, String>();

        Options options = new Options();
        options.addOption(ROBOT_NUMBER_ARG_NAME, true, "number of robots");
        options.addOption(ROBOT_POSE_RADIUS_ARG_NAME, true, "pose radius for robots");
        options.addOption(ROS_IP_ARG_NAME, true, "local ip or hostname");
        options.addOption(ROS_MASTER_RUI_ARG_NAME, true, "ros master url");

        CommandLineParser commandLineParser = new BasicParser();
        try {
            CommandLine cmd = commandLineParser.parse(options, args);
            String n = cmd.getOptionValue(ROBOT_NUMBER_ARG_NAME);
            conf.put(ROBOT_NUMBER_KEY, n);
            String r = cmd.getOptionValue(ROBOT_POSE_RADIUS_ARG_NAME);
            conf.put(ROBOT_POSE_RADIUS_KEY, r);
            String ip = cmd.getOptionValue(ROS_IP_ARG_NAME);
            conf.put(ROS_IP_KEY, ip);
            String master = cmd.getOptionValue(ROS_MASTER_RUI_ARG_NAME);
            conf.put(ROS_MASTER_RUI_KEY, master);
            return conf;
        } catch (ParseException e) {
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp("simulator", options);
        }
        return null;
    }
}
