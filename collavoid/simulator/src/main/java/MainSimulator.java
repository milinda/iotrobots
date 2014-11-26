import nav_msgs.Odometry;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import simbad.gui.Simbad;
import simbad.sim.*;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

public class MainSimulator {


    private static ConnectedNode node;

    /** Describe the robot */
    static public class Robot extends Agent {

        RangeSensorBelt sonars;
        CameraSensor camera;

        Publisher<Odometry> odometryPublisher;
        Odometry odoMsg;
        int seq;


        public Robot(Vector3d position, String name) {
            super(position, name);
            // Add camera
            camera = RobotFactory.addCameraSensor(this);
            // Add sonars
            sonars = RobotFactory.addSonarBeltSensor(this);
        }

        /** This method is called by the simulator engine on reset. */
        public void initBehavior() {
            // initialize publisher
            if (odometryPublisher==null) {
                odometryPublisher = node.newPublisher(this.getName()+"/Odometry",Odometry._TYPE);
                 // initialize message
                odoMsg = utils.messageFactory.newFromType(Odometry._TYPE);
                odoMsg.getHeader().setFrameId(this.getName() + "/Odometry");
                odoMsg.setChildFrameId(this.getName() + "/base");
                seq = 0;
            }
            this.rotateY(Math.PI/3);
        }

        /** This method is call cyclically (20 times per second)  by the simulator engine. */
        public void performBehavior() {

            // progress at 0.5 m/s
            //if (getCounter()==1)
            //setTranslationalVelocity(0.1);
            // frequently change orientation

//            if (getCounter()==500){
//                setTranslationalVelocity(0);
//                translateTo(new Vector3d(3,0,3));
//            }
//
//            if (getCounter()==510){
//                setRotationalVelocity(0.5);
//            }

//            if ((getCounter() % 100) == 0)
//                setRotationalVelocity(Math.PI / 2 * (0.5 - Math.random()));
//
//            // print front sonar every 100 frames
//            if (getCounter() % 100 == 0)
//                System.out
//                        .println("Sonar num 0  = " + sonars.getMeasurement(0));

            if(getCounter()%20==0){
                setOdoMsg(this);
                odometryPublisher.publish(odoMsg);
            }

        }

        public void setOdoMsg(Robot robot){
            //get position
            Point3d cor=new Point3d();
            robot.getCoords(cor);

            //get orientation
            Transform3D tfr=new Transform3D();
            Quat4d ori=new Quat4d();
            robot.getRotationTransform(tfr);
            tfr.get(ori);

            odoMsg.getHeader().setStamp(node.getCurrentTime());
            odoMsg.getHeader().setSeq(seq++);

            odoMsg.getPose().setPose(utils.getPose(cor,ori));
            odoMsg.getTwist().getTwist().setLinear(utils.getTwist(robot.linearVelocity));
            odoMsg.getTwist().getTwist().setAngular(utils.getTwist(robot.angularVelocity));

        }

    }

    /** Describe the environement */
    static public class MyEnv extends EnvironmentDescription {
        public MyEnv(ConnectedNode node) {
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
            add(new Arch(new Vector3d(3, 0, -3), this));

            for (int i = 0; i <5 ; i++) {
                add(new Robot(new Vector3d(0, 0, i), "robot"+i));

            }


        }
    }

    public static void main(String[] args) {

        odoPub odometryPublishNode=new odoPub("agent_odo_pub");
        node=odometryPublishNode.getNode();
        // request antialising
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new MyEnv(odometryPublishNode.getNode()), false);

    }

} 