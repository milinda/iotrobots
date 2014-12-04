import cgl.iotrobots.collavoid.GlobalPlanner.GlobalPlanner;
import cgl.iotrobots.collavoid.LocalPlanner.LocalPlanner;
import geometry_msgs.PoseStamped;
import org.ros.node.ConnectedNode;
import org.ros.rosjava.tf.pubsub.TransformListener;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by hjh on 11/30/14.
 */
public class InitPlannerThread extends Thread {
    //planner
    GlobalPlanner globalPlanner = null;
    LocalPlanner localPlanner = null;
    List<PoseStamped> globalPlan = null;
    ConnectedNode node;
    TransformListener tfl;
    Point3d start;
    Point3d goal;
    Quat4d oriGoal;

    InitPlannerThread(ConnectedNode node, TransformListener tfl, Point3d start, Point3d goal, Quat4d oriGoal) {
        super("InitPlannerThread");
        this.node = node;
        this.tfl = tfl;
        this.start = start;
        this.goal = goal;
        this.oriGoal = oriGoal;

        start();

    }

    public void run() {
        //initialize plan
        if (globalPlan == null) {
            globalPlan = new ArrayList<PoseStamped>();
        } else {
            globalPlan.clear();
        }

        if (globalPlanner == null)
            globalPlanner = new GlobalPlanner();

        if (localPlanner == null)
            localPlanner = new LocalPlanner(node, tfl);

        start = utils.toROSCoordinate(start);
        goal = utils.toROSCoordinate(goal);
        utils.toROSCoordinate(oriGoal);

        PoseStamped startPose = node.getTopicMessageFactory().newFromType(PoseStamped._TYPE);
        PoseStamped goalPose = node.getTopicMessageFactory().newFromType(PoseStamped._TYPE);

        startPose.getPose().getPosition().setX(start.getX());
        startPose.getPose().getPosition().setY(start.getY());

        goalPose.getPose().getPosition().setX(goal.getX());
        goalPose.getPose().getPosition().setY(goal.getY());
        goalPose.getPose().getOrientation().setX(oriGoal.getX());
        goalPose.getPose().getOrientation().setY(oriGoal.getY());
        goalPose.getPose().getOrientation().setZ(oriGoal.getZ());
        goalPose.getPose().getOrientation().setW(oriGoal.getW());

        globalPlanner.makePlan(startPose, goalPose, globalPlan);

        if (!localPlanner.setPlan(globalPlan))
            node.getLog().error("Set global plan error!");

    }
}
