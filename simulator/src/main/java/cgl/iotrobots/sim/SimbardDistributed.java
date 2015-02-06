package cgl.iotrobots.sim;

import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.slam.core.app.GFSAlgorithm;
import cgl.iotrobots.slam.core.app.GFSMap;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.gridfastsalm.GridSlamProcessor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.streaming.Utils;
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
    public static final double ANGLE_RANGE_SIDE = 0.521567881107;

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
//        private String url = "amqp://localhost:5672";
        private String url = "amqp://156.56.93.59:5672";

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

        /** This method is called by the simulator engine on reset. */
        public void initBehavior() {
            SimUtils.sendControl(controlSender);
        }

        boolean forward = false;

        long lastTime = System.currentTimeMillis();

        /** This method is call cyclically (20 times per second)  by the simulator engine. */
        public void performBehavior() {
//            System.out.println("\n\n");
            Point3d point3D = new Point3d(0.0, 0.0, 0.0);
            getCoords(point3D);

            //System.out.format("actual position: %f, %f, %f\n", point3D.x, point3D.y, point3D.z);
            Quat4d trs = getOrientation();
            //System.out.format("actual position: %f, %f, %f %f\n", trs.getX(), trs.getY(), trs.getZ(), trs.getW());
            double theta = quantarianToRad(new Quaternion(trs.getX(), trs.getZ(), trs.getY(), trs.getW()));
            //System.out.format("theta %f\n", theta);
            System.out.format("actual position: %f, %f, %f, %f\n", point3D.x, point3D.y, point3D.z, theta * 2);
            LaserScan laserScan = getLaserScan();
            laserScan.setPose(new DoubleOrientedPoint(point3D.x, -point3D.z, theta * 2));

            byte []body = Utils.serialize(kryo, laserScan);
            Map<String, Object> props = new HashMap<String, Object>();
            props.put("time", System.currentTimeMillis());
            props.put(TransportConstants.SENSOR_ID, System.currentTimeMillis());

            if (System.currentTimeMillis() - lastTime > 3000) {
                lastTime = System.currentTimeMillis();
                Message message = new Message(body, props);
                try {
                    sender.send(message, "test.test.laser_scan");
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            // progress at 0.5 m/s
            if (getCounter() % 500 == 0) {
                forward = !forward;
            }

            if (forward) {
                setTranslationalVelocity(.5);
            } else {
                setTranslationalVelocity(-.5);
            }

            if ((getCounter() % 2) == 0)
                setRotationalVelocity(Math.PI / 2 * (- Math.random()) / 2);
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
//                GFSMap map = (GFSMap) Utils.deSerialize(kryo, message.getBody(), GFSMap.class);
                Object time = message.getProperties().get("time");
                Long t = Long.parseLong(time.toString());
                bestSum += System.currentTimeMillis() - t;
                bestCount++;
                System.out.println("*******************Best Time: " + (System.currentTimeMillis() - t) + "Average: " + ((double)(bestSum) / bestCount) +" ***************************");
//                mapUI.setMap(map);
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

        private double quantarianToRad(Quaternion q) {
            return new Matrix3(q).getEulerYPR().yaw;
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
            laserScan.setAngleMax(ANGLE);
            laserScan.setAngleMin(0);
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
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new MyEnv(), false);

        showOnScreen(1, frame);
        mapUI = new MapUI();
//        showOnScreen(1, mapUI);
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

    public static class Matrix3 {
        double [][]m_el = new double[3][3];

        private Matrix3(Quaternion q) {
            setRotation(q);
        }

        void setRotation(Quaternion q) {
            double d = 2;
            double s = 2.0 / d;
            double xs = q.getX() * s,   ys = q.getY() * s,   zs = q.getZ() * s;
            double wx = q.getW() * xs,  wy = q.getW() * ys,  wz = q.getW() * zs;
            double xx = q.getX() * xs,  xy = q.getX() * ys,  xz = q.getX() * zs;
            double yy = q.getY() * ys,  yz = q.getY() * zs,  zz = q.getZ() * zs;

            setValue((1.0) - (yy + zz), xy - wz, xz + wy,
                    xy + wz, (1.0) - (xx + zz), yz - wx,
                    xz - wy, yz + wx, (1.0) - (xx + yy));
        }

        void setValue(double xx, double xy, double xz,
                      double yx, double yy, double yz,
                      double zx, double zy, double zz) {
            m_el[0][0] = xx;
            m_el[0][1] = xy;
            m_el[0][2] = xz;
            m_el[1][0] = yx;
            m_el[1][1] = yy;
            m_el[1][2] = yz;
            m_el[2][0] = zx;
            m_el[2][1] = zy;
            m_el[2][2] = zz;
        }

        Euler getEulerYPR()
        {
            Euler euler_out = new Euler();
            Euler euler_out2 = new Euler(); //second solution
            //get the pointer to the raw data

            // Check that pitch is not at a singularity
            // Check that pitch is not at a singularity
            if (Math.abs(m_el[2][0]) >= 1)
            {
                euler_out.yaw = 0;
                euler_out2.yaw = 0;

                // From difference of angles formula
                if (m_el[2][0] < 0)  //gimbal locked down
                {
                    double delta = Math.atan2(m_el[0][1],m_el[0][2]);
                    euler_out.pitch = Math.PI / 2.0;
                    euler_out2.pitch = Math.PI / 2.0;
                    euler_out.roll = delta;
                    euler_out2.roll = delta;
                }
                else {
                    double delta = Math.atan2(-m_el[0][1],-m_el[0][2]);
                    euler_out.pitch = -Math.PI / 2.0;
                    euler_out2.pitch = -Math.PI / 2.0;
                    euler_out.roll = delta;
                    euler_out2.roll = delta;
                }
            }
            else
            {
                euler_out.pitch = - Math.asin(m_el[2][0]);
                euler_out2.pitch = Math.PI - euler_out.pitch;

                euler_out.roll = Math.atan2(m_el[2][1]/Math.cos(euler_out.pitch),
                        m_el[2][2]/Math.cos(euler_out.pitch));
                euler_out2.roll = Math.atan2(m_el[2][1]/Math.cos(euler_out2.pitch),
                        m_el[2][2]/Math.cos(euler_out2.pitch));

                euler_out.yaw = Math.atan2(m_el[1][0]/Math.cos(euler_out.pitch),
                        m_el[0][0]/Math.cos(euler_out.pitch));
                euler_out2.yaw = Math.atan2(m_el[1][0]/Math.cos(euler_out2.pitch),
                        m_el[0][0]/Math.cos(euler_out2.pitch));
            }

            Euler e = new Euler();

            e.yaw = euler_out.yaw;
            e.pitch = euler_out.pitch;
            e.roll = euler_out.roll;

            return e;
        }
    }

    static class Euler {
        double yaw;
        double pitch;
        double roll;
    }
}
