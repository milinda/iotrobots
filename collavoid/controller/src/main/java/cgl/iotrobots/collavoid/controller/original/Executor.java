package cgl.iotrobots.collavoid.controller.original;

import cgl.iotrobots.collavoid.commons.planners.Parameters;
import org.ros.node.NodeConfiguration;

import java.util.ArrayList;
import java.util.List;

public class Executor {
    private static final int RobotNB = Parameters.ROBOT_NUMBER;
    private static List<AgentController> controllers = new ArrayList<AgentController>();

    public static void main(String[] args) {
        for (int i = 0; i < RobotNB; i++) {
            controllers.add(new AgentController("robot" + i + "_rmq", null, "amqp://localhost:5672"));// use
        }
        NodeConfiguration configuration = NodeConfiguration.newPublic("localhost");
        for (AgentController ac : controllers)
            ac.start(configuration);

        Runtime.getRuntime().addShutdownHook(new Thread() {
            public void run() {
                for (AgentController ac : controllers)
                    ac.stop();
            }
        });
    }
}
