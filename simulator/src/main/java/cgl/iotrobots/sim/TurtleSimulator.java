package cgl.iotrobots.sim;

import cgl.iotrobots.slam.core.app.GFSAlgorithm;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.gridfastsalm.GridSlamProcessor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.threading.ParallelGridSlamProcessor;
import cgl.iotrobots.slam.utils.Matrix3;
import cgl.iotrobots.slam.utils.RosMapPublisher;
import geometry_msgs.Quaternion;
import nav_msgs.Odometry;
import org.apache.commons.lang3.tuple.Pair;

import java.io.BufferedReader;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;

public class TurtleSimulator {
    public static final int SENSORS = 360;

    public static final double ANGLE = 2 * Math.PI;

    GFSAlgorithm gfsAlgorithm = new GFSAlgorithm();

    BufferedReader br = null;

    int parallel = 2;

    MapUI mapUI;

    RosMapPublisher rosMapPublisher = new RosMapPublisher();

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

        RosTurtle rosTurtle = new RosTurtle(this);
        SimUtils.connectToRos(rosTurtle);
        SimUtils.connectToRos(rosMapPublisher);

        Thread t = new Thread(new TurtleSimulator.Worker(rosTurtle.getQueue()));
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
        private BlockingQueue<Pair<Odometry, sensor_msgs.LaserScan>> queue;

        private Worker(BlockingQueue<Pair<Odometry, sensor_msgs.LaserScan>> queue) {
            this.queue = queue;
        }

        @Override
        public void run() {
            while (true) {
                try {
                    Pair<Odometry, sensor_msgs.LaserScan> pair = null;
                    while (queue.size() > 0) {
                        pair = queue.take();
                    }
                    if (pair != null) {
                        LaserScan scan = turtleScanToLaserScan(pair.getRight());
                        Odometry odometry = pair.getLeft();
                        double rad = quantarianToRad(odometry.getPose().getPose().getOrientation());
                        System.out.format("received odometry: x = %f, y = %f, theta = %f\n", odometry.getPose().getPose().getPosition().getX(), odometry.getPose().getPose().getPosition().getY(), rad);
                        DoubleOrientedPoint lastPose = new DoubleOrientedPoint(odometry.getPose().getPose().getPosition().getX(),
                                odometry.getPose().getPose().getPosition().getY(), rad);

                        scan.setPose(lastPose);
                        gfsAlgorithm.laserScan(scan);

                        mapUI.setMap(gfsAlgorithm.getMap());
                        rosMapPublisher.addMap(gfsAlgorithm.getMap());
                    }
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
        double angle = ls.getAngleMin();
        if (ls.getRanges() != null) {
            List<Double> ranges = new ArrayList<Double>();
            float[] floats = ls.getRanges();
            for (float r : floats) {
                if (angle < Math.toRadians(23) && angle > Math.toRadians(-23)) {
                    if (Float.isNaN(r)) {
                        ranges.add((double) ls.getRangeMax());
                    }
                    ranges.add((double) r);
                }
                angle += ls.getAngleIncrement();
            }
            laserScan.setRanges(ranges);
        }
        return laserScan;
    }

    private DoubleOrientedPoint getOdomPose(long t) {
        return null;
    }


}
