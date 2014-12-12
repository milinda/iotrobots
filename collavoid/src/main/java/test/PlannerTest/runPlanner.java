package test.PlannerTest;

import cgl.iotrobots.collavoid.GlobalPlanner.GlobalPlanner;
import cgl.iotrobots.collavoid.LocalPlanner.LocalPlanner;
import cgl.iotrobots.collavoid.utils.ROSAgentNode;
import geometry_msgs.PoseStamped;
import geometry_msgs.Twist;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.rosjava.tf.Transform;
import org.ros.rosjava.tf.pubsub.TransformListener;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by hjh on 12/9/14.
 */
public class runPlanner {
    static int robotNb=2;
    static List<Planner> planners=new ArrayList<Planner>();
    static ConnectedNode node;
    static ParameterTree params;
    static TransformListener tfl = null;

    public static void main(String[] args){
        ROSAgentNode agentNode=new ROSAgentNode("tfnode");
        node=agentNode.getNode();
        params=node.getParameterTree();
        params.set("initializedPlanners",0);
        robotNb=params.getInteger("robotNb");
        tfl=new TransformListener(node);

        for (int i = 0; i < robotNb; i++) {
            planners.add(new Planner(tfl,i));
        }
    }
}

