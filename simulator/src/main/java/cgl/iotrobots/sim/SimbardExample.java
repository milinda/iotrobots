package cgl.iotrobots.sim;

import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.app.GFSAlgorithm;
import cgl.iotrobots.slam.core.gridfastsalm.GridSlamProcessor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.threading.ParallelGridSlamProcessor;
import simbad.gui.Simbad;
import simbad.sim.*;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

public class SimbardExample {
    public static final int SENSORS = 360;

    public static final double ANGLE = 2 * Math.PI;

    static MapUI mapUI;
    /** Describe the robot */
    static public class Robot extends Agent {
        GFSAlgorithm gfsAlgorithm = new GFSAlgorithm();
        RangeSensorBelt sonars;
        CameraSensor camera;

        PrintWriter pw;

        public Robot(Vector3d position, String name) {
            super(position, name);
            // Add camera
            camera = RobotFactory.addCameraSensor(this);
            // Add sonars
            double agentHeight = this.getHeight();
            double agentRadius = this.getRadius();
            sonars = new RangeSensorBelt((float) agentRadius,
                    0f, 100.0f, SENSORS, RangeSensorBelt.TYPE_SONAR,0);
            sonars.setUpdatePerSecond(1000);

            Vector3d pos = new Vector3d(0, agentHeight / 2, 0.0);
            this.addSensorDevice(sonars, pos, 0);

            try {
                pw = new PrintWriter(new FileWriter("out.txt"));
            } catch (IOException e) {
                e.printStackTrace();
            }

        }

        /** This method is called by the simulator engine on reset. */
        public void initBehavior() {
            // nothing particular in this case
            gfsAlgorithm.gsp_ = new ParallelGridSlamProcessor();
            gfsAlgorithm.init();
            LaserScan scanI = new LaserScan();
            scanI.setAngle_increment(ANGLE / SENSORS);
            scanI.setAngle_max(ANGLE);
            scanI.setAngle_min(0);
            List<Double> ranges  = new ArrayList<Double>();
            for (int i = 0; i < SENSORS; i++) {
                ranges.add(100.0);
            }
            scanI.setRanges(ranges);
            scanI.setRange_min(0);
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

            System.out.format("%f, %f, %f\n", point3D.x, point3D.y, point3D.z);
            LaserScan laserScan = getLaserScan();
            laserScan.setPose(new DoubleOrientedPoint(point3D.x, 0.0, 0.0));
            pw.printf("%s\n", laserScan.getString());
            System.out.println(laserScan.getString());
            gfsAlgorithm.laserScan(laserScan);
            prevX = point3D.x;
            // progress at 0.5 m/s
            if (getCounter() % 60 == 0) {
                if (forward) {
                    forward = false;
                } else {
                    forward = true;
                }
            }

            if (forward) {
                setTranslationalVelocity(5);
            } else {
                setTranslationalVelocity(-5);
            }
            // frequently change orientation
//            if ((getCounter() % 100) == 0)
//                setRotationalVelocity(Math.PI / 2 * (0.5 - Math.random()));

            mapUI.setMap(gfsAlgorithm.getMap());

            // print front sonar every 100 frames
//            if (getCounter() % 100 == 0)
//                System.out
//                        .println("Sonar num 0  = " + sonars.getMeasurement(0));
        }

        private LaserScan getLaserScan() {
            int n = sonars.getNumSensors();

            LaserScan laserScan = new LaserScan();
            laserScan.setAngle_max(ANGLE);
            laserScan.setAngle_min(0);
            laserScan.setRangeMax(100);
            laserScan.setRange_min(0);
            laserScan.setAngle_increment(ANGLE/ SENSORS);

            int angle = 0;
            List<Double> ranges  = new ArrayList<Double>();
            for (int i = angle; i < n; i++) {
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
            add(new Robot(new Vector3d(-5, 0, 0), "robot 1"));
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

}
