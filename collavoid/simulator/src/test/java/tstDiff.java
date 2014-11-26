import simbad.demo.Demo;
import simbad.gui.Simbad;
import simbad.sim.Agent;
import simbad.sim.Box;
import simbad.sim.DifferentialKinematic;
import simbad.sim.RobotFactory;

import javax.vecmath.Color3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

/**
 * Created by hjh on 11/24/14.
 */
public class tstDiff {


    /**
     * A differential drive (two wheels) kinematic  demo. This demo demonstrates the usage of
     * the DifferentialKinematic instead of the standard one.
     */
    static public class DifferentialKinematicDemo extends Demo {

        public class Robot extends Agent {
            DifferentialKinematic kinematic;
            int state;

            public Robot(Vector3d position, String name) {
                super(position, name);
                // Change the  kinematic to differentialDrive
                kinematic = RobotFactory.setDifferentialDriveKinematicModel(this);
            }

            /**
             * Initialize Agent's Behavior
             */
            public void initBehavior() {
                state = 0;
            }

            /**
             * Perform one step of Agent's Behavior
             */
            public void performBehavior() {

                // here we do not use SetTranslationalVelocity and SetRotationalVelocity.
                // Instead we control directly the wheels velocity.
                // We need to use  the kinematic object to do so.

                if (getCounter() % 300 == 0) {

                    switch (state) {

                        // turn right
                        case 0:
                            kinematic.setWheelsVelocity(0.8, 0.5);
                            break;
                        // turn left
                        case 1:
                            kinematic.setWheelsVelocity(0.5, 0.8);
                            break;

                        case 2:
                            kinematic.setWheelsVelocity(-0.8, 0.5);
                            break;
                        // on place
                        case 3:
                            kinematic.setWheelsVelocity(0.8, 0);
                            break;
                        // forward
                        case 4:
                            kinematic.setWheelsVelocity(0.1, 0.1);
                            break;
                        // backward
                        case 5:
                            kinematic.setWheelsVelocity(-0.5, -0.5);
                            state = -1;
                            break;

                    }
                    state++;
                }

                if(getCounter()%100==0){
                    System.out.println("ang"+this.angularVelocity.toString());
                    System.out.println("linear"+this.linearVelocity.toString());
                }

                if (collisionDetected()) moveToStartPosition();
            }

        }

        public DifferentialKinematicDemo() {
            boxColor = new Color3f(0.6f, 0.5f, .3f);

            add(new Box(new Vector3d(-8, 0, 0), new Vector3f(0.1f, 1, 16), this));
            add(new Box(new Vector3d(0, 0, -8), new Vector3f(16, 1, 0.1f), this));
            add(new Box(new Vector3d(8, 0, 0), new Vector3f(0.1f, 1, 16), this));
            add(new Box(new Vector3d(0, 0, 8), new Vector3f(16, 1, 0.1f), this));


            // Add a robot.
            add(new Robot(new Vector3d(0, 0, 0), "my robot"));


        }
    }

    public static void main(String[] args) {
        //need to set VM option as -Djava.library.path="/home/hjh/software/j3d-1_5_2-linux-amd64/lib/amd64"

        // request antialising
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new DifferentialKinematicDemo(), false);

    }
}