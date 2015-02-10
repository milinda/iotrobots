package cgl.iotrobots.sim;

import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.app.GFSAlgorithm;
import cgl.iotrobots.slam.core.gridfastsalm.GridSlamProcessor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.utils.RosMapPublisher;
import cgl.iotrobots.slam.utils.TurtleUtils;
import simbad.gui.Simbad;
import simbad.sim.*;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

public class SimbardExample {
    public static final double ANGLE_RANGE_SIDE = 0.521567881107;

    public static final int SENSORS = 640;

    public static final double ANGLE = 2 * Math.PI;

    static MapUI mapUI;
    /** Describe the robot */
    static public class Robot extends Agent {
        GFSAlgorithm gfsAlgorithm = new GFSAlgorithm();
        RangeSensorBelt sonars;
        PrintWriter pw;

        int totalSensors = 0;

        RosMapPublisher node = new RosMapPublisher();

        public Robot(Vector3d position, String name) {
            super(position, name);
            // Add sonars
            double agentHeight = this.getHeight();
            double agentRadius = this.getRadius();

            totalSensors = (int) Math.floor(2 * Math.PI * SENSORS / (ANGLE_RANGE_SIDE * 2));
            sonars = new RangeSensorBelt((float) agentRadius,
                    .1f, 100.0f, totalSensors, RangeSensorBelt.TYPE_SONAR,0);
            sonars.setUpdatePerSecond(100);

            Vector3d pos = new Vector3d(0, agentHeight / 2, 0.0);
            this.addSensorDevice(sonars, pos, 0);

//            try {
//                pw = new PrintWriter(new FileWriter("out.txt"));
//            } catch (IOException e) {
//                e.printStackTrace();
//            }

            TurtleUtils.connectToRos(node);

        }

        public void initBehavior() {
            gfsAlgorithm.gsp_ = new GridSlamProcessor();
            gfsAlgorithm.init();
            LaserScan scanI = new LaserScan();
            scanI.setAngleIncrement(ANGLE / totalSensors);
            scanI.setAngleMax(ANGLE_RANGE_SIDE);
            scanI.setAngleMin(-ANGLE_RANGE_SIDE);
            List<Double> ranges  = new ArrayList<Double>();
            for (int i = 0; i < SENSORS; i++) {
                ranges.add(100.0);
            }
            scanI.setRanges(ranges);
            scanI.setRangeMin(.1);
            scanI.setRangeMax(100);
            scanI.setTimestamp(System.currentTimeMillis());

            gfsAlgorithm.initMapper(scanI);
        }

        boolean forward = false;
        double prevX = 0;
        /** This method is call cyclically (20 times per second)  by the simulator engine. */
        public void performBehavior() {
            System.out.println("\n\n");
            Point3d point3D = new Point3d(0.0, 0.0, 0.0);
            getCoords(point3D);

            Quat4d trs = getOrientation();
//            double theta = quantarianToRad(new Quaternion(trs.getX(), trs.getZ(), trs.getY(), trs.getW()));
            double theta = getYaw(new Quaternion(trs.getX(), trs.getZ(), trs.getY(), trs.getW()));
            System.out.format("actual position: %f, %f, %f\n", point3D.x, -point3D.z, theta);
//            Transform3D trs = getTransform();
//            System.out.format("actual position: %f, %f, %f %f\n", trs.getX(), trs.getY(), trs.getZ());
            LaserScan laserScan = getLaserScan();

            DoubleOrientedPoint noisyPoint = new DoubleOrientedPoint(point3D.x + TurtleUtils.gausianNoise(.0005, 0), -point3D.z + TurtleUtils.gausianNoise(.0005, 0), theta + TurtleUtils.gausianNoise(.0005, 0));
            System.out.println("Noisy point: " + noisyPoint);
            laserScan.setPose(noisyPoint);
            for (int i = 0; i < laserScan.getRanges().size(); i++) {
                System.out.format("%f ", laserScan.getRanges().get(i));
            }
            System.out.format("\n");

            gfsAlgorithm.laserScan(laserScan);
            prevX = point3D.x;

            if (getCounter() % 500 == 0) {
                forward = !forward;
            }

            if (forward) {
                setTranslationalVelocity(.25);
            } else {
                setTranslationalVelocity(-.25);
            }

            if ((getCounter() % 2) == 0)
                setRotationalVelocity(Math.PI / 2 * (- Math.random()) / 2);

            mapUI.setMap(gfsAlgorithm.getMap());
            node.addMap(gfsAlgorithm.getMap());
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

        // local planner related
        public static double getYaw(Quaternion q) {
            double q0 = q.getX();
            double q1 = q.getY();
            double q2 = q.getZ();
            double q3 = q.getW();
            //refer to roll in http://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
            return Math.atan2(2.0 * (q0 * q1 + q3 * q2), q3 * q3 + q0 * q0 - q1 * q1 - q2 * q2);
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
        mapUI = new MapUI();
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

    private double quantarianToRad(Quaternion q) {
        //return Math.atan2(2.0 * (q.getY() * q.getZ() + q.getW() * q.getX()), q.getW() * q.getW() - q.getX() * q.getX() - q.getY() * q.getY() + q.getZ() * q.getZ());
//        return Math.atan2(2.0 * (q.getX() * q.getZ() + q.getY() * q.getW()), 1 - 2 * (q.getZ() * q.getZ() + q.getW() * q.getW()));
        return new Matrix3(q).getEulerYPR().yaw;
    }

}
