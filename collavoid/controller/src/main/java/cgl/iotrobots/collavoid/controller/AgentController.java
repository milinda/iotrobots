package cgl.iotrobots.collavoid.controller;

import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.rmqmsg.RMQContext;
import com.rabbitmq.client.*;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.util.HashMap;
import java.util.Map;

public class AgentController {

    private AgentROSNode agentROSNode;

    private NodeMainExecutor nodeMainExecutor;

    private Channel channel;
    
    private Address[] addresses;

    private String url;

    private String exchangeName;

    private String nodeName;

    private Map<String, RMQContext> RMQContexts=new HashMap<String, RMQContext>();

    public AgentController(String Name,
                           Address[] addresses,
                           String url) {
        this.exchangeName = Name;
        this.nodeName = Name;
        this.addresses = addresses;
        this.url = url;

        // information from robot use the same exchange
        RMQContexts.put(Constant_msg.KEY_ODOMETRY, new RMQContext(exchangeName, Constant_msg.KEY_ODOMETRY));
        RMQContexts.put(Constant_msg.KEY_PARTICLE_CLOUD, new RMQContext(exchangeName, Constant_msg.KEY_PARTICLE_CLOUD));
        RMQContexts.put(Constant_msg.KEY_SCAN, new RMQContext(exchangeName, Constant_msg.KEY_SCAN));
        RMQContexts.put(Constant_msg.KEY_VELOCITY_CMD, new RMQContext(exchangeName, Constant_msg.KEY_VELOCITY_CMD));
    }

    public void start(NodeConfiguration configuration) {
        channel = Methods_RMQ.getChannel(addresses, url, null, exchangeName, Constant_msg.TYPE_EXCHANGE_DIRECT);
        agentROSNode = new AgentROSNode(nodeName, channel, RMQContexts);
        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(agentROSNode, configuration);
    }

    public void stop() {
//        try {
//            if (channel != null) {
//                channel.close();
//            }
//            if (connection != null) {
//                connection.close();
//            }
            if (agentROSNode != null) {
                nodeMainExecutor.shutdown();
            }
//        } catch (IOException e) {
//            System.out.println("Error closing the rabbit MQ connection" + e);
//        }
    }

}
