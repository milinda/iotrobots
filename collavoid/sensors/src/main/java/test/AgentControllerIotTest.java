package test;

import cgl.iotcloud.core.SensorContext;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class AgentControllerIotTest {
    private SensorContext context;
    private AgentROSNodeIotTest agentROSNode;
    private String AgentID;
    private NodeMainExecutor nodeMainExecutor;

    public AgentControllerIotTest(SensorContext context) {
        this.context = context;
        this.AgentID = "robot" + context.getProperty(Constant_storm.IotCloud.AGENT_INDEX);
    }

    public void start(NodeConfiguration configuration) {
        //need to be a different node name
        agentROSNode = new AgentROSNodeIotTest(AgentID + "_rmq", context);
        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(agentROSNode, configuration);

    }
}
