package cgl.iotrobots.collavoid.controller;

import com.rabbitmq.client.*;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;
import java.util.Map;
import java.util.concurrent.ExecutorService;

public class AgentController {

    private AgentROSNode agentROSNode;

    private NodeMainExecutor nodeMainExecutor;

    private Channel channel;

    private Connection connection;

    private Address[] addresses;

    private String url;

    private ExecutorService executorService;

    private String exchangeName;

    private String nodeName;

    private Map<String, String> RMQParams;

    public AgentController(String Name,
                           Address[] addresses,
                           String url) {
        this.exchangeName = Name;
        this.nodeName = Name;
        this.addresses = addresses;
        this.url = url;
        RMQParams.put(Constants.KEY_ROUTINGKEY_VELOCITY, Constants.KEY_ROUTINGKEY_VELOCITY);
        RMQParams.put(Constants.KEY_ROUTINGKEY_ODOMETRY, Constants.KEY_ROUTINGKEY_ODOMETRY);
        RMQParams.put(Constants.KEY_ROUTINGKEY_SCAN, Constants.KEY_ROUTINGKEY_SCAN);
        RMQParams.put(Constants.KEY_ROUTINGKEY_PARTICLE, Constants.KEY_ROUTINGKEY_PARTICLE);
        RMQParams.put(Constants.KEY_EXCHAGE_NAME, Name);
    }

    public void start(NodeConfiguration configuration) {

        setChannel();
        agentROSNode = new AgentROSNode(nodeName, channel, RMQParams);
        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(agentROSNode, configuration);
    }

    private void setChannel() {
        try {
            ConnectionFactory factory = new ConnectionFactory();
            if (addresses == null) {
                factory.setUri(url);
                if (executorService != null) {
                    connection = factory.newConnection(executorService);
                } else {
                    connection = factory.newConnection();
                }
            } else {
                if (executorService != null) {
                    connection = factory.newConnection(executorService, addresses);
                } else {
                    connection = factory.newConnection(addresses);
                }
            }

            channel = connection.createChannel();

            if (exchangeName != null) {
                channel.exchangeDeclare(exchangeName, "direct", true);
            }

        } catch (IOException e) {
            String msg = "Error consuming the message";
            throw new RuntimeException(msg, e);
        } catch (Exception e) {
            String msg = "Error connecting to broker";
            throw new RuntimeException(msg, e);
        }

    }

    public void stop() {
        try {
            if (channel != null) {
                channel.close();
            }
            if (connection != null) {
                connection.close();
            }
            if (agentROSNode != null) {
                nodeMainExecutor.shutdown();
            }
        } catch (IOException e) {
            System.out.println("Error closing the rabbit MQ connection" + e);
        }
    }

}
