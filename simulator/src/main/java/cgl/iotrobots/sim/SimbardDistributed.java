package cgl.iotrobots.sim;

import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.slam.core.app.GFSMap;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.streaming.Utils;
import cgl.iotrobots.slam.utils.TurtleUtils;
import cgl.iotrobots.utils.rabbitmq.*;
import com.esotericsoftware.kryo.Kryo;
import simbad.gui.Simbad;
import simbad.sim.*;
import simbad.sim.Box;

import javax.media.j3d.Transform3D;
import javax.swing.*;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import java.awt.*;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


public class SimbardDistributed {
//    public static final double ANGLE_RANGE_SIDE = 0.521567881107;
    public static final double ANGLE_RANGE_SIDE = Math.PI / 2;

    public static final int SENSORS = 640;

    public static final double ANGLE = 2 * Math.PI;

    static MapUI mapUI;
    /** Describe the robot */
    static public class Robot extends Agent {
        RangeSensorBelt sonars;
        CameraSensor camera;
        RabbitMQSender sender;
        RabbitMQReceiver receiver;
        RabbitMQReceiver bestReceiver;
        RabbitMQSender controlSender;

        Kryo kryo = new Kryo();

        PrintWriter pw;
//        private String url = "amqp://149.165.159.12:5672";
        private String url = "amqp://localhost:5672";
//        private String url = "amqp://156.56.93.59:5672";

        int totalSensors = 0;

        public Robot(Vector3d position, String name) {
            super(position, name);

            try {
                controlSender = new RabbitMQSender(url, "simbard_control");
                sender = new RabbitMQSender(url, "simbard_laser");
                receiver = new RabbitMQReceiver(url, "simbard_map");
                bestReceiver = new RabbitMQReceiver(url, "simbard_best");
                sender.open();
                controlSender.open();
                receiver.listen(new MapReceiver());
                bestReceiver.listen(new BestParticleReceiver());
            } catch (Exception e) {
                e.printStackTrace();
            }
            // Add camera
            camera = RobotFactory.addCameraSensor(this);
            // Add sonars
            double agentHeight = this.getHeight();
            double agentRadius = this.getRadius();
            totalSensors = (int) Math.floor(2 * Math.PI * SENSORS / (ANGLE_RANGE_SIDE * 2));
            sonars = new RangeSensorBelt((float) agentRadius,
                    .1f, 100.0f, totalSensors, RangeSensorBelt.TYPE_SONAR,0);
            sonars.setUpdatePerSecond(100);

            Vector3d pos = new Vector3d(0, agentHeight / 2, 0.0);
            this.addSensorDevice(sonars, pos, 0);

        }

        public void initBehavior() {
            TurtleUtils.sendControl(controlSender);
        }

        boolean forward = false;

        long lastTime = System.currentTimeMillis();

        public void performBehavior() {
            Point3d point3D = new Point3d(0.0, 0.0, 0.0);
            getCoords(point3D);
            Quat4d trs = getOrientation();
            double theta = getYaw(new Quaternion(trs.getX(), trs.getZ(), trs.getY(), trs.getW()));
            System.out.format("actual position: %f, %f, %f, %f\n", point3D.x, point3D.y, point3D.z, theta);
            LaserScan laserScan = getLaserScan();
            laserScan.setPose(new DoubleOrientedPoint(point3D.x + Math.random() / 10, -point3D.z + Math.random() / 10, theta + Math.random() / 10));

            byte []body = Utils.serialize(kryo, laserScan);
            Map<String, Object> props = new HashMap<String, Object>();
            props.put("time", System.currentTimeMillis());
            props.put(TransportConstants.SENSOR_ID, System.currentTimeMillis());

            if (System.currentTimeMillis() - lastTime > 500) {
                lastTime = System.currentTimeMillis();
                Message message = new Message(body, props);
                try {
                    sender.send(message, "test.test.laser_scan");
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            if (getCounter() % 200 == 0) {
                forward = !forward;
            }

            if (forward) {
                setTranslationalVelocity(.1);
            } else {
                setTranslationalVelocity(-.1);
            }

            if ((getCounter() % 2) == 0)
                setRotationalVelocity(- Math.random() / 10);
        }

        private long bestSum;
        private long mapSum;
        private long bestCount;
        private long mapCount;

        private class BestParticleReceiver implements MessageHandler {
            @Override
            public Map<String, String> getProperties() {
                Map<String, String> props = new HashMap<String, String>();
                props.put(MessagingConstants.RABBIT_ROUTING_KEY, "test.test.best");
                props.put(MessagingConstants.RABBIT_QUEUE, "test.test.best");
                return props;
            }

            @Override
            public void onMessage(Message message) {
                Object time = message.getProperties().get("time");
                Long t = Long.parseLong(time.toString());
                bestSum += System.currentTimeMillis() - t;
                bestCount++;
                System.out.println("*******************Best Time: " + (System.currentTimeMillis() - t) + "Average: " + ((double)(bestSum) / bestCount) +" ***************************");
            }
        }

        private class MapReceiver implements MessageHandler {
            @Override
            public Map<String, String> getProperties() {
                Map<String, String> props = new HashMap<String, String>();
                props.put(MessagingConstants.RABBIT_ROUTING_KEY, "test.test.map");
                props.put(MessagingConstants.RABBIT_QUEUE, "test.test.map");
                return props;
            }

            @Override
            public void onMessage(Message message) {
                GFSMap map = (GFSMap) Utils.deSerialize(kryo, message.getBody(), GFSMap.class);
                Object time = message.getProperties().get("time");
                Long t = Long.parseLong(time.toString());
                mapCount++;
                mapSum += (System.currentTimeMillis() - t);
                System.out.println("*******************Map Time: " + (System.currentTimeMillis() - t) + "Average: " + ((double)(mapSum) / mapCount) +" ***************************");
                mapUI.setMap(map);
            }
        }

        // local planner related
        public static double getYaw(Quaternion q) {
            double q0 = q.getX();
            double q1 = q.getY();
            double q2 = q.getZ();
            double q3 = q.getW();
            //refer to roll in http://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
            return Math.atan2(2.0 * (q0 * q1 + q3 * q2), q3 * q3 + q0 * q0 - q1 * q1 - q2 * q2);
        }

        public Quat4d getOrientation() {
            //get orientation
            Transform3D tfr = new Transform3D();
            Quat4d ori = new Quat4d();
            this.getRotationTransform(tfr);
            tfr.get(ori);
            return ori;
        }

        private LaserScan getLaserScan() {
            LaserScan laserScan = new LaserScan();
            laserScan.setAngleMax(ANGLE_RANGE_SIDE);
            laserScan.setAngleMin(-ANGLE_RANGE_SIDE);
            laserScan.setRangeMax(100);
            laserScan.setRangeMin(.1);
            laserScan.setAngleIncrement(ANGLE / totalSensors);

            int angle = totalSensors - SENSORS / 2;
            List<Double> ranges  = new ArrayList<Double>();
            for (int i = angle; i < totalSensors; i++) {
                if (sonars.hasHit(i)) {
                    ranges.add(sonars.getMeasurement(i));
                } else {
                    ranges.add(0.0);
                }
            }
            for (int i = 0; i < SENSORS / 2; i++) {
                if (sonars.hasHit(i)) {
                    ranges.add(sonars.getMeasurement(i));
                } else {
                    ranges.add(0.0);
                }
            }

            laserScan.setRanges(ranges);
            laserScan.setTimestamp(System.currentTimeMillis());
            return laserScan;
        }
    }

    /** Describe the environement */
    static public class MyEnv extends EnvironmentDescription {
        public MyEnv() {
            light1IsOn = true;
            light2IsOn = false;
            Wall w1 = new Wall(new Vector3d(9, 0, 0), 19, 1, this);
            w1.rotate90(1);
            add(w1);
            Wall w2 = new Wall(new Vector3d(-9, 0, 0), 19, 2, this);
            w2.rotate90(1);
            add(w2);
            Wall w3 = new Wall(new Vector3d(0, 0, 9), 19, 1, this);
            add(w3);
            Wall w4 = new Wall(new Vector3d(0, 0, -9), 19, 2, this);
            add(w4);
            Box b1 = new Box(new Vector3d(-3, 0, -3), new Vector3f(1, 1, 1),
                    this);
            add(b1);

            Box b2 = new Box(new Vector3d(3, 0, 3), new Vector3f(1, 1, 1),
                    this);
            add(b2);

            Box b3 = new Box(new Vector3d(6, 0, 6), new Vector3f(1, 1, 1),
                    this);
            add(b3);

            Box b4 = new Box(new Vector3d(-6, 0, -6), new Vector3f(1, 1, 1),
                    this);
            add(b4);
            add(new Arch(new Vector3d(3, 0, -3), this));
            add(new Robot(new Vector3d(0, 0, 0), "robot 1"));
            //add(new Robot(new Vector3d(0, 0, 0), "robot 2"));
        }
    }

    public static void main(String[] args) {
        // request antialising
        mapUI = new MapUI();
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new MyEnv(), false);
        showOnScreen(1, frame);
    }

    public static void showOnScreen( int screen, JFrame frame )
    {
        GraphicsEnvironment ge = GraphicsEnvironment
                .getLocalGraphicsEnvironment();
        GraphicsDevice[] gs = ge.getScreenDevices();
        if (screen > -1 && screen < gs.length) {
            gs[screen].setFullScreenWindow(frame);
        } else if (gs.length > 0) {
            gs[0].setFullScreenWindow(frame);
        } else {
            throw new RuntimeException("No Screens Found");
        }
    }

    public static class Quaternion {
        double x, y, z, w;

        private Quaternion() {
        }

        private Quaternion(double x, double y, double z, double w) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getZ() {
            return z;
        }

        public double getW() {
            return w;
        }
    }
}
