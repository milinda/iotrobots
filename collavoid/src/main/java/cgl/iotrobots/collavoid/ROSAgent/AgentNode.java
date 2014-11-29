package cgl.iotrobots.collavoid.ROSAgent;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

/**
 * Created by hjh on 11/28/14.
 */
public class AgentNode extends AbstractNodeMain {
    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("Agent");
    }
}
