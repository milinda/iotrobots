package cgl.iotrobots.sim;

import cgl.iotrobots.slam.core.sample.LaserScan;
import cgl.iotrobots.slam.core.sample.Sample;
import cgl.iotrobots.slam.core.utils.OrientedPoint;
import simbad.gui.Simbad;
import simbad.sim.*;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import java.util.ArrayList;

public class Example1 {
    static MapUI mapUI;
    /** Describe the robot */
    static public class Robot extends Agent {
        Sample sample = new Sample();
        RangeSensorBelt sonars;
        CameraSensor camera;

        public Robot(Vector3d position, String name) {
            super(position, name);
            // Add camera
            camera = RobotFactory.addCameraSensor(this);
            // Add sonars
//            sonars = RobotFactory.addSonarBeltSensor(this, 16);

            double agentHeight = this.getHeight();
            double agentRadius = this.getRadius();
            sonars = new RangeSensorBelt((float) agentRadius,
                    0f, 100.0f, 512, RangeSensorBelt.TYPE_SONAR,0);
            sonars.setUpdatePerSecond(3);
            //sonarBelt.setName("sonars");
            Vector3d pos = new Vector3d(0, agentHeight / 2, 0.0);
            this.addSensorDevice(sonars, pos, 0);

        }

        /** This method is called by the simulator engine on reset. */
        public void initBehavior() {
            // nothing particular in this case
            sample.init();
            LaserScan scanI = new LaserScan();
            scanI.angle_increment = 2 * Math.PI / 512;
            scanI.angle_max = 2*Math.PI ;
            scanI.angle_min = 0;
            scanI.ranges = new ArrayList<Double>();
            for (int i = 0; i < 512; i++) {
                scanI.ranges.add(10.0);
            }
            scanI.range_min = 0;
            scanI.range_max = 10;

            sample.initMapper(scanI);
        }

        /** This method is call cyclically (20 times per second)  by the simulator engine. */
        public void performBehavior() {
            Point3d point3D = new Point3d(0.0, 0.0, 0.0);
            getCoords(point3D);
            // System.out.println(point3D.x + " " + point3D.y + " " + point3D.z);
            LaserScan laserScan = getLaserScan();
            sample.laserCallback(laserScan, new OrientedPoint<Double>(point3D.x, point3D.y, point3D.z));
            // progress at 0.5 m/s
            setTranslationalVelocity(1);
            // frequently change orientation
            if ((getCounter() % 100) == 0)
                setRotationalVelocity(Math.PI / 2 * (0.5 - Math.random()));

            mapUI.setMap(sample.map_);

            // print front sonar every 100 frames
            if (getCounter() % 100 == 0)
                System.out
                        .println("Sonar num 0  = " + sonars.getMeasurement(0));
                sample.printMap(sample.map_);

        }

        LaserScan getLaserScan() {
            int n = sonars.getNumSensors();

            LaserScan laserScan = new LaserScan();
            laserScan.angle_max = Math.PI * 2;
            laserScan.angle_min = 0;
            laserScan.range_max = 10;
            laserScan.range_min = 0;
            laserScan.angle_increment = 2 * Math.PI / 512;

//            for (double angle = 0; angle < 2 * Math.PI / 2; angle += 2 * Math.PI / n) {
//                int hits = sonars.getQuadrantHits(angle, angle + 2 * Math.PI / n);
//                double val = 0;
//                if (hits > 0) {
//                    val = sonars.getQuadrantMeasurement(angle, angle + 2 * Math.PI / n);
//                    System.out.println("Laser value: " + val);
//                } else {
//                    System.out.println("No object in range");
//                }
//                laserScan.ranges.add(val);
//            }
            for (int i = 0; i < n; i++) {
                if (sonars.hasHit(i)) {
                    // System.out.println(sonars.getMeasurement(i));
                    laserScan.ranges.add(sonars.getMeasurement(i));
                } else {
                    laserScan.ranges.add(0.0);
                }
            }
            laserScan.timestamp = System.currentTimeMillis();

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
        mapUI = new MapUI();
    }

}
