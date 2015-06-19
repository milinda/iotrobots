package cgl.iotrobots.collavoid.simulator;

import org.ros.namespace.GraphName;
import org.ros.node.*;

import java.net.URI;
import java.net.URISyntaxException;

public class AgentNode {
    private ConnectedNode node;
    private NodeMainExecutor executor;
    private String nodeName;

    public class PubSubNode extends AbstractNodeMain {
        private boolean initialized = false;

        @Override
        public void onStart(ConnectedNode connectedNode) {
            node = connectedNode;
            initialized = true;
        }

        @Override
        public GraphName getDefaultNodeName() {
            return GraphName.of(nodeName);
        }

    }

    public AgentNode(String nodeName, String rosip, String rosMaster) {
        NodeConfiguration nodeConfiguration = null;
        try {
            nodeConfiguration = NodeConfiguration.newPublic(rosip, new URI(rosMaster));
        } catch (URISyntaxException e) {
            e.printStackTrace();
        }
        this.executor = DefaultNodeMainExecutor.newDefault();
        this.nodeName = nodeName;
        nodeConfiguration.setNodeName(nodeName);

        PubSubNode pubSubNode = new PubSubNode();
        executor.execute(pubSubNode, nodeConfiguration);

        System.out.println("Initializing Node " + nodeName + "..");

        int i = 0;
        while (!pubSubNode.initialized) {
            i++;
            System.out.print(".");
            if (i % 30 == 0)
                System.out.println();
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        System.out.println("Done!");
    }

    public ConnectedNode getNode() {
        if (node != null) {
            return node;
        } else {
            System.out.println("Error, node not initialized!");
            return null;
        }
    }

    public void shutdown() {
        executor.shutdown();
    }


}
