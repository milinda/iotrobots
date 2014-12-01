import org.ros.namespace.GraphName;
import org.ros.node.*;

/**
 * Created by hjh on 11/25/14.
 */
public class AgentNode {

    private PubSubNode pubSubNode;
    private ConnectedNode node;

    public class PubSubNode extends AbstractNodeMain {
        private boolean initialized = false;

        @Override
        public void onStart(ConnectedNode connectedNode) {
            node = connectedNode;
            initialized = true;
        }

        @Override
        public GraphName getDefaultNodeName() {
            return GraphName.of("agent");
        }

    }

    public AgentNode(String nodeName) {

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic("localhost");
        final NodeMainExecutor executor = DefaultNodeMainExecutor.newDefault();
        nodeConfiguration.setNodeName(nodeName);

        pubSubNode = new PubSubNode();
        executor.execute(pubSubNode, nodeConfiguration);

        System.out.println("Initializing Node " + nodeName + "..");

        while (!pubSubNode.initialized) {
            System.out.print(".");
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        System.out.println("Done!");
    }

    public ConnectedNode getNode() {
        if (pubSubNode.initialized) {
            return node;
        } else {
            System.out.println("Error, node not initialized!");
            return null;
        }
    }


}
