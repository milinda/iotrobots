package test.PlannerTest;

import cgl.iotrobots.collavoid.utils.ROSAgentNode;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.rosjava.tf.pubsub.TransformListener;

import java.util.ArrayList;
import java.util.List;


public class runPlanner {
    static int robotNb = 2;
    static List<Planner> planners = new ArrayList<Planner>();
    static ConnectedNode node;
    static ParameterTree params;
    static TransformListener tfl = null;

    public static void main(String[] args) {
        ROSAgentNode agentNode = new ROSAgentNode("tfnode");
        node = agentNode.getNode();
        params = node.getParameterTree();
        params.set("initializedPlanners", 0);
        robotNb = params.getInteger("robotNb");
        tfl = new TransformListener(node);

        for (int i = 0; i < robotNb; i++) {
            planners.add(new Planner(tfl, i));
        }
    }
}

