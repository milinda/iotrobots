import cgl.iotcloud.core.AbstractConfigurator;
import cgl.iotcloud.core.SensorContext;

import org.ros.node.NodeConfiguration;

import java.net.URI;
import java.net.URISyntaxException;


public class testnode {
    public static SensorContext context;

    public static void main(String[] args) {

        AgentControllerIot controllerIot = new AgentControllerIot(context);
        NodeConfiguration nodeConfiguration = null;
        try {
            nodeConfiguration = NodeConfiguration.newPublic("localhost", new URI("http://localhost:11311"));
        } catch (URISyntaxException e) {

        }

        controllerIot.start(nodeConfiguration);

    }

}
