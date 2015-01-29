package cgl.iotrobots.collavoid.simulator;

import cgl.iotrobots.collavoid.commons.planners.Parameters;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import com.rabbitmq.client.AMQP;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;
import simbad.gui.Simbad;
import simbad.sim.*;

import javax.media.j3d.Transform3D;
import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;


public class SimulatorNoRos {
    static final int robotNb = Parameters.ROBOT_NUMBER;
    static final double posRadius = Parameters.POSE_RADIUS;
    static List<Robot> robots = new ArrayList<Robot>();

    //test
    static int reachGoalNo = 0;
    static int startedNo = 0;
    //test

    static public class Robot extends Agent {
        //test
        long lastNewVelTime;
        boolean startAgain;
        //test

        //id
        int id;
        String robotName;
        //orientation
        double orientation;
        //kinematic
        DifferentialKinematic kinematic;
        private double wheelDistance;
        //sensors
        LaserScan laserScan;
        RangeSensorBelt sensors;
        RangeSensorBelt bumpers;
        //pub and sub
        private Channel channel;
        private Map<String, RMQContext> RMQContexts;

        PoseStamped_ start, goal;
        BaseConfig_ baseConfig_;
        int sgseq;

        Odometry_ odomMsg;
        PointCloud2_ pc2;
        PoseArray_ poseArray;

        private double vl, vr;
        long time;

        //frame
        String robotFrame;
        String odomFrame;
        String globalFrame;
        Point3d previousPosition = null;

        public Robot(Vector3d position, double ori, int id) {
            // initialize position and orientation
            super(position, "robot" + id);
            this.robotName = "robot" + id;
            this.id = id;
            this.radius = (float) Parameters.FOOTPRINT_RADIUS;// loaded from agent parameters
            orientation = ori;
            //use differential model
            kinematic = RobotFactory.setDifferentialDriveKinematicModel(this);
            wheelDistance = this.getRadius();
            // Add sensors
            bumpers = RobotFactory.addBumperBeltSensor(this, 12);
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
            RMQContexts = new Contexts(robotName).getRMQContexts();
            channel = Methods_RMQ.getChannel(null, Constant_msg.RMQ_URL, null);
            try {
                for (Map.Entry<String, RMQContext> e : RMQContexts.entrySet()) {
                    e.getValue().CHANNEL = channel;
                    e.getValue().CHANNEL.exchangeDeclare(
                            e.getValue().EXCHANGE_NAME,
                            e.getValue().EXCHANGE_TYPE,
                            e.getValue().DURABLE
                    );
                    if (e.getKey().equals(Constant_msg.KEY_VELOCITY_CMD)) {
                        e.getValue().CHANNEL.queueDeclare(e.getValue().QUEUE_NAME, false, false, true, null);
                        e.getValue().CHANNEL.queueBind(
                                e.getValue().QUEUE_NAME,
                                e.getValue().EXCHANGE_NAME,
                                e.getValue().ROUTING_KEY
                        );
                        e.getValue().CHANNEL.queuePurge(e.getValue().QUEUE_NAME);
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
            odomMsg = new Odometry_();
            odomMsg.setId(robotName);
            odomMsg.getHeader().setFrameId(odomFrame);
            odomMsg.setChildFrameId(robotFrame);

            // initialize message
            pc2 = new PointCloud2_();
            pc2.setId(robotName);
            pc2.getHeader().setFrameId(globalFrame);

            poseArray = new PoseArray_();
            poseArray.setId(robotName);
            poseArray.getHeader().setFrameId(robotFrame);

            //initialize velocity command subscriber
            try {
                boolean autoAck = false;
                String queueName = RMQContexts.get(Constant_msg.KEY_VELOCITY_CMD).QUEUE_NAME;
                String routingKey = RMQContexts.get(Constant_msg.KEY_VELOCITY_CMD).ROUTING_KEY;
                final Channel velCmdChannel = RMQContexts.get(Constant_msg.KEY_VELOCITY_CMD).CHANNEL;
                velCmdChannel.basicConsume(queueName, autoAck, routingKey + "Tag",
                        new DefaultConsumer(velCmdChannel) {
                            @Override
                            public void handleDelivery(String consumerTag,
                                                       Envelope envelope,
                                                       AMQP.BasicProperties properties,
                                                       byte[] body)
                                    throws IOException {
                                long deliveryTag = envelope.getDeliveryTag();
                                Twist_ velocity = (Twist_) Methods_RMQ.deSerialize(body, Twist_.class);
                                double v, w;
                                //in ros coordinate do not need to transform the coordinate
                                v = velocity.getLinear().length();
                                w = velocity.getAngular().getZ();
                                vl = v - w * wheelDistance / 2;
                                vr = v + w * wheelDistance / 2;
//                                System.out.println(robotName+" cmd delay:"+(System.currentTimeMillis()-lastNewVelTime));
//                                System.out.println(System.currentTimeMillis()-lastNewVelTime);
                                velCmdChannel.basicAck(deliveryTag, false);
                                lastNewVelTime = System.currentTimeMillis();
                                //test
                                if (velocity.isGoalReached()) {
                                    reachGoalNo++;
                                    startAgain = true;
                                    startedNo = 0;
                                }
                                //test
                            }
                        });
            } catch (IOException e) {
                String msg = "Error consuming the message";
                throw new RuntimeException(msg, e);
            } catch (Exception e) {
                String msg = "Error connecting to broker";
                throw new RuntimeException(msg, e);
            }

            start = new PoseStamped_();
            start.getHeader().setFrameId(globalFrame);
            goal = new PoseStamped_();
            goal.getHeader().setFrameId(globalFrame);

            baseConfig_ = new BaseConfig_();
            baseConfig_.setId(robotName);
            baseConfig_.setPublisMeFreq(Parameters.PUBLISH_ME_FREQUENCY);
            baseConfig_.setControlFreq(Parameters.CONTROLLER_FREQUENCY);
            baseConfig_.setStart(start);
            baseConfig_.setGoal(goal);
            sgseq = 0;
        }

        /**
         * This method is called by the simulator engine on reset.
         */
        public void initBehavior() {
            Methods_RMQ.clearQueues(RMQContexts);
            this.resetPosition();
            this.rotateY(orientation);
            sgseq = 0;
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
            if (sgseq < 10) {
                kinematic.setWheelsVelocity(0, 0);
            } else {
                kinematic.setWheelsVelocity(vl, vr);
            }

            time = System.currentTimeMillis();

            //publish scan in frequency of 10Hz
            if (getCounter() % 1 == 0) {
                pc2.getHeader().setStamp(time);
                //publish valid laser scan in pointcloud2 format in global frame
                laserScan.getLaserscanPointCloud2(pc2, this.getTransform());
                Methods_RMQ.publishMsg(RMQContexts.get(Constant_msg.KEY_SCAN),
                        Methods_RMQ.serialize(pc2));
            }
            //test publish localization pose array
            if (getCounter() % 2 == 0) {
                poseArray.getHeader().setStamp(time);
                Point3d cor = new Point3d();
                this.getCoords(cor);
                if (previousPosition == null || !cor.equals(previousPosition)) {
                    if (previousPosition == null)
                        previousPosition = new Point3d();
                    this.getCoords(previousPosition);
                    setPoseArrayMsg();
                }
                Methods_RMQ.publishMsg(RMQContexts.get(Constant_msg.KEY_POSE_ARRAY),
                        Methods_RMQ.serialize(poseArray));
            }

            if (getCounter() % 1 == 0) {
                setOdomMsg(time);
                Methods_RMQ.publishMsg(RMQContexts.get(Constant_msg.KEY_ODOMETRY),
                        Methods_RMQ.serialize(odomMsg));
            }

            if (sgseq < 10) {
                start.getHeader().setStamp(time);
                goal.getHeader().setStamp(time);
                getStartGoal();
                baseConfig_.setTime(time);

                Methods_RMQ.publishMsg(RMQContexts.get(Constant_msg.KEY_BASE_CONFIG),
                        Methods_RMQ.serialize(baseConfig_));
                sgseq++;
            }

            //test
            if (reachGoalNo >= robotNb && startAgain) {
                sgseq = 0;
                this.startAgain = false;
                if (++startedNo >= robotNb)
                    reachGoalNo = 0;
//                System.out.println(robotName+"reached goal");
            }
            if (bumpers.oneHasHit()) {
                System.out.println("Collision detected: " + robotName);
            }
            //test

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
            start.setPose(utilsSimNoRos.getPose(start_cor, oriStart));
            goal.setPose(utilsSimNoRos.getPose(goal_cor, oriGoal));
        }

        public void setOdomMsg(long t) {
            //get position and velocities, all in odometry frame
            Point3d cor = new Point3d();
            this.getCoords(cor);
            Quat4d ori = getOrientation();

            odomMsg.getHeader().setStamp(t);
            odomMsg.setPose(utilsSimNoRos.getPose(cor, ori));
            odomMsg.getTwist().setLinear(utilsSimNoRos.getTwist(this.linearVelocity));
            odomMsg.getTwist().setAngular(utilsSimNoRos.getTwist(this.angularVelocity));

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
            List<Pose_> pa = new ArrayList<>();
            //get orientation
            Transform3D tfr = new Transform3D();
            this.getRotationTransform(tfr);

            Quat4d ori = new Quat4d();
            Point3d pt = new Point3d();
            for (int i = 0; i < 50; i++) {
                // in robot base frame
                pt.setX(utilsSimNoRos.getGaussianNoise(0, radius / 10));
                pt.setZ(utilsSimNoRos.getGaussianNoise(0, radius / 10));
                tfr.rotY(utilsSimNoRos.getGaussianNoise(0, 0.15));
                tfr.get(ori);
                pa.add(utilsSimNoRos.getPose(pt, ori));
            }
            poseArray.setPoses(pa);
        }

        public void shutDown() {
            try {
                for (Map.Entry<String, RMQContext> context : RMQContexts.entrySet()) {
                    if (!context.getValue().CHANNEL.isOpen())
                        continue;
                    if (context.getKey().equals(Constant_msg.KEY_VELOCITY_CMD))
                        context.getValue().CHANNEL.queueDelete(context.getValue().QUEUE_NAME);
//                    context.getValue().CHANNEL.exchangeDelete(context.getValue().EXCHANGE_NAME);
                }
                channel.close();

            } catch (IOException e) {
                System.out.println("Error closing the rabbit MQ connection" + e);
            }
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

//            there are bugs in avoid obstacles
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
