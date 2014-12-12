import cgl.iotrobots.collavoid.GlobalPlanner.GlobalPlanner;
import geometry_msgs.PoseStamped;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by hjh on 12/4/14.
 */
public class tstGlobalPlanner {
    static Point3d start,goal;
    static Quat4d oriGoal;
    static List<PoseStamped> globalPlan=new ArrayList<PoseStamped>();
    private static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    private static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);
    static GlobalPlanner globalPlanner=new GlobalPlanner();

    public static void main(String[] args) {

        //set start point and goal
        start = new Point3d(6,0,0);
        goal = new Point3d();
        goal.set(-start.getX(), start.getY(), -start.getZ());

        oriGoal=new Quat4d(0,0,0,1);
        Transform3D tfr = new Transform3D(oriGoal, new Vector3d(0, 0, 0), 1);
        tfr.rotZ(Math.PI);
        tfr.get(oriGoal);

        PoseStamped startPose = messageFactory.newFromType(PoseStamped._TYPE);
        PoseStamped goalPose = messageFactory.newFromType(PoseStamped._TYPE);

        startPose.getHeader().setFrameId("map");
        startPose.getPose().getPosition().setX(start.getX());
        startPose.getPose().getPosition().setY(start.getY());

        goalPose.getHeader().setFrameId("map");
        goalPose.getPose().getPosition().setX(goal.getX());
        goalPose.getPose().getPosition().setY(goal.getY());
        goalPose.getPose().getOrientation().setX(oriGoal.getX());
        goalPose.getPose().getOrientation().setY(oriGoal.getY());
        goalPose.getPose().getOrientation().setZ(oriGoal.getZ());
        goalPose.getPose().getOrientation().setW(oriGoal.getW());

        globalPlanner.makePlan(startPose, goalPose, globalPlan);
        for (int i = 0; i <globalPlan.size() ; i++) {
            System.out.println(globalPlan.get(i).getPose().getPosition().getX()+";"+globalPlan.get(i).getPose().getPosition().getY());
        }
    }
}
