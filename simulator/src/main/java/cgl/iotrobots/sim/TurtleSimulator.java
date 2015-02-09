package cgl.iotrobots.sim;

import cgl.iotrobots.slam.core.app.GFSAlgorithm;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.gridfastsalm.GridSlamProcessor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.threading.ParallelGridSlamProcessor;
import geometry_msgs.Quaternion;
import nav_msgs.Odometry;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.*;
import org.ros.node.topic.Subscriber;

import java.io.BufferedReader;
import java.util.ArrayList;
import java.util.List;

public class TurtleSimulator {
    public static final int SENSORS = 360;

    public static final double ANGLE = 2 * Math.PI;

    GFSAlgorithm gfsAlgorithm = new GFSAlgorithm();

    BufferedReader br = null;

    int parallel = 2;

    MapUI mapUI;

    RosMapPublisher node = new RosMapPublisher();

    public TurtleSimulator() {
        mapUI = new MapUI();
    }

    public void start(boolean parallel) throws InterruptedException {
        // nothing particular in this case
        if (!parallel) {
            gfsAlgorithm.gsp_ = new GridSlamProcessor();
        } else {
            gfsAlgorithm.gsp_ = new ParallelGridSlamProcessor();
        }
        gfsAlgorithm.init();


        SimUtils.connectToRos(new RosTurtle());

        SimUtils.connectToRos(node);

        Thread t = new Thread(new Worker());
        t.start();
    }

    public static void main(String[] args) throws InterruptedException {
        TurtleSimulator simulator = new TurtleSimulator();
        if (args.length > 0) {
            simulator.start(true);
            simulator.parallel = Integer.parseInt(args[0]);
        } else {
            simulator.start(false);
        }
    }

    public class RosTurtle extends AbstractNodeMain {
        private String name = "/ts_controller";

        private RosMapPublisher rosMapPublisher;

        public RosTurtle() {
        }

        public GraphName getDefaultNodeName() {
            return GraphName.of("/" + name);
        }

        @Override
        public void onStart(final ConnectedNode connectedNode) {
            System.out.println("Starting....");
            final Subscriber<Odometry> odometrySubscriber =
                    connectedNode.newSubscriber("/odom", Odometry._TYPE);

            final Subscriber<sensor_msgs.LaserScan> laserScanSubscriber =
                    connectedNode.newSubscriber("/scan", sensor_msgs.LaserScan._TYPE);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ie) {
                ie.printStackTrace();
            }

            odometrySubscriber.addMessageListener(new MessageListener<Odometry>() {
                @Override
                public void onNewMessage(Odometry odometry) {
                    double rad = quantarianToRad(odometry.getPose().getPose().getOrientation());
                    if (state == State.WAITING_LASER_SCAN) {
                        System.out.format("received odometry: x = %f, y = %f, theta = %f\n", odometry.getPose().getPose().getPosition().getX(), odometry.getPose().getPose().getPosition().getY(), rad);
                        lastPose = new DoubleOrientedPoint(odometry.getPose().getPose().getPosition().getX(),
                                odometry.getPose().getPose().getPosition().getY(), rad);
                    }
                }
            });

            laserScanSubscriber.addMessageListener(new MessageListener<sensor_msgs.LaserScan>() {
                @Override
                public void onNewMessage(sensor_msgs.LaserScan laserScanMsg) {
                    if (state == State.WAITING_LASER_SCAN) {
                        state = State.UPDATING_LASER_SCAN;
                        scan = turtleScanToLaserScan(laserScanMsg);
                        state = State.WAITING_COMPUTING;
                    }
                }
            });
        }

        public void onShutdown(Node node) {
            node.shutdown();
        }
    }

    DoubleOrientedPoint lastPose;
    enum State {
        UPDATING_LASER_SCAN,
        WAITING_COMPUTING,
        COMPUTING,
        WAITING_LASER_SCAN,
    }
    LaserScan scan;
    State state = State.WAITING_LASER_SCAN;
    boolean init = false;

    private void init(LaserScan scanI) {
        gfsAlgorithm.initMapper(scanI);
    }

    private double quantarianToRad(Quaternion q) {
        //return Math.atan2(2.0 * (q.getY() * q.getZ() + q.getW() * q.getX()), q.getW() * q.getW() - q.getX() * q.getX() - q.getY() * q.getY() + q.getZ() * q.getZ());
//        return Math.atan2(2.0 * (q.getX() * q.getZ() + q.getY() * q.getW()), 1 - 2 * (q.getZ() * q.getZ() + q.getW() * q.getW()));
        return new Matrix3(q).getEulerYPR().yaw;
    }

    private class Worker implements Runnable {
        @Override
        public void run() {
            while (true) {
                if (state == State.WAITING_COMPUTING) {
                    state = State.COMPUTING;
                    System.out.println("Start computing......................");
                    if (scan != null && lastPose != null) {
                        if (!init) {
                            init(scan);
                            init = true;
                        } else {
                            scan.setPose(lastPose);
                            gfsAlgorithm.laserScan(scan);
                            mapUI.setMap(gfsAlgorithm.getMap());
                            node.addMap(gfsAlgorithm.getMap());
                        }
                    }
                    state = State.WAITING_LASER_SCAN;
                    System.out.println("End computing......................");
                }
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }


    private LaserScan turtleScanToLaserScan(sensor_msgs.LaserScan ls) {
        LaserScan laserScan = new LaserScan();
        laserScan.setAngleIncrement(ls.getAngleIncrement());
        laserScan.setAngleMin(ls.getAngleMin());
        laserScan.setAngleMax(ls.getAngleMax());
        laserScan.setRangeMax(ls.getRangeMax());
        laserScan.setRangeMin(ls.getRangeMin());
        laserScan.setTimestamp((long) ls.getScanTime());
        if (ls.getRanges() != null) {
            List<Double> ranges = new ArrayList<Double>();
            float[] floats = ls.getRanges();
            for (float r : floats) {
                if (Float.isNaN(r)) {
                    ranges.add((double) ls.getRangeMax());
                }
                ranges.add((double) r);
            }
            laserScan.setRanges(ranges);
        }
        return laserScan;
    }

    private DoubleOrientedPoint getOdomPose(long t) {
        return null;
    }

    private class Matrix3 {
        double[][] m_el = new double[3][3];

        private Matrix3(Quaternion q) {
            setRotation(q);
        }

        void setRotation(Quaternion q) {
            double d = 2;
            double s = 2.0 / d;
            double xs = q.getX() * s, ys = q.getY() * s, zs = q.getZ() * s;
            double wx = q.getW() * xs, wy = q.getW() * ys, wz = q.getW() * zs;
            double xx = q.getX() * xs, xy = q.getX() * ys, xz = q.getX() * zs;
            double yy = q.getY() * ys, yz = q.getY() * zs, zz = q.getZ() * zs;

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

        Euler getEulerYPR() {
            Euler euler_out = new Euler();
            Euler euler_out2 = new Euler(); //second solution
            //get the pointer to the raw data

            // Check that pitch is not at a singularity
            // Check that pitch is not at a singularity
            if (Math.abs(m_el[2][0]) >= 1) {
                euler_out.yaw = 0;
                euler_out2.yaw = 0;

                // From difference of angles formula
                if (m_el[2][0] < 0)  //gimbal locked down
                {
                    double delta = Math.atan2(m_el[0][1], m_el[0][2]);
                    euler_out.pitch = Math.PI / 2.0;
                    euler_out2.pitch = Math.PI / 2.0;
                    euler_out.roll = delta;
                    euler_out2.roll = delta;
                } else {
                    double delta = Math.atan2(-m_el[0][1], -m_el[0][2]);
                    euler_out.pitch = -Math.PI / 2.0;
                    euler_out2.pitch = -Math.PI / 2.0;
                    euler_out.roll = delta;
                    euler_out2.roll = delta;
                }
            } else {
                euler_out.pitch = -Math.asin(m_el[2][0]);
                euler_out2.pitch = Math.PI - euler_out.pitch;

                euler_out.roll = Math.atan2(m_el[2][1] / Math.cos(euler_out.pitch),
                        m_el[2][2] / Math.cos(euler_out.pitch));
                euler_out2.roll = Math.atan2(m_el[2][1] / Math.cos(euler_out2.pitch),
                        m_el[2][2] / Math.cos(euler_out2.pitch));

                euler_out.yaw = Math.atan2(m_el[1][0] / Math.cos(euler_out.pitch),
                        m_el[0][0] / Math.cos(euler_out.pitch));
                euler_out2.yaw = Math.atan2(m_el[1][0] / Math.cos(euler_out2.pitch),
                        m_el[0][0] / Math.cos(euler_out2.pitch));
            }

            Euler e = new Euler();

            e.yaw = euler_out.yaw;
            e.pitch = euler_out.pitch;
            e.roll = euler_out.roll;

            return e;
        }
    }

    class Euler {
        double yaw;
        double pitch;
        double roll;
    }


}
