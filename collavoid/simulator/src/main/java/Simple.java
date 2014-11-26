import simbad.gui.Simbad;
import simbad.sim.*;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import java.util.ArrayList;
import java.util.List;

public class Simple {


    /** Describe the robot */
    static public class Robot extends Agent {

        CmdVelListener velListener;

        DifferentialKinematic kinematic;
        RangeSensorBelt sensors;
        LaserScan laserScan;
        CameraSensor camera;

        public Robot(Vector3d position, String name) {
            //robot center is in the center of the cylinder
            super(position, name);

            //use differential model
            kinematic = RobotFactory.setDifferentialDriveKinematicModel(this);

            // Add camera
            camera = RobotFactory.addCameraSensor(this);
            // Add sensors
            laserScan=new LaserScan(this.getRadius(),Math.PI,180,1.5,3);
            sensors=laserScan.getSensor();

            // if sensors are not centered the robot coordinate then height
            // should be assigned to the laserscan
            this.addSensorDevice(sensors,new Vector3d(0,0,0),0);
          }

        /** This method is called by the simulator engine on reset. */
        public void initBehavior() {
            // create listening node
            velListener=new CmdVelListener(this.getRadius(),this.getName());

            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

        /** This method is call cyclically (20 times per second)  by the simulator engine. */
        public void performBehavior() {

            kinematic.setLeftVelocity(velListener.getVl());
            kinematic.setRightVelocity(velListener.getVr());
            Vector3d angularVel=this.angularVelocity;

            Transform3D tft=new Transform3D();
            Transform3D tfr=new Transform3D();
            Quat4d qt=new Quat4d();
            Vector3d tt=new Vector3d();
            Quat4d qr=new Quat4d();
            Vector3d tr=new Vector3d();
            Point3d coord= new Point3d();
            List<Point3d> scan=new ArrayList<Point3d>();

            // progress at 0.5 m/s
            //setTranslationalVelocity(0.5);
            // frequently change orientation
            if ((getCounter() % 100) == 0)
            //    setRotationalVelocity(Math.PI / 2 * (0.5 - Math.random()));

            // print front sonar every 100 frames
            if (getCounter() % 100 == 0) {
                System.out
                        .println("Sonar num 0  = " + sensors.getMeasurement(0));
                this.getTranslationTransform(tft);
                this.getRotationTransform(tfr);
                this.getCoords(coord);
                tft.get(qt,tt);
                tfr.get(qr,tr);
                //System.out.println(tt.toString()+"\n"+coord.toString()+"\n"+this.getHeight()+"\n");
               // System.out.println(qr.toString()+"    "+Math.acos(qr.getW())*2+"    "+Math.PI/4);
                scan=laserScan.getScan();
                for (int i = 0; i <scan.size() ; i++) {
                    Point3d pt=new Point3d();
                    tfr.transform(scan.get(i),pt);
                    tft.transform(pt);
                    System.out.println(i + " map frame:" + pt.toString());
                    tft.invert();
                    tft.transform(pt);
                    tfr.invert();
                    tfr.transform(pt);
                    System.out.println("robot frame"+pt.toString());
                }

            }

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
            Box b2 = new Box(new Vector3d(1, 0, 0), new Vector3f(1, 1, 1),
                    this);
            add(b1);
            add(b2);
            add(new Arch(new Vector3d(3, 0, -3), this));

            add(new Robot(new Vector3d(0, 0, 0), "robot1"));

        }
    }


    public static void main(String[] args) {
        //need to set VM option as -Djava.library.path="/home/hjh/software/j3d-1_5_2-linux-amd64/lib/amd64"

        // request antialising
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new MyEnv(), false);

    }

} 