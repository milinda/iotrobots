package cgl.iotrobots.collavoid.planners;

import cgl.iotrobots.collavoid.commons.rmqmsg.MsgContext;
import com.rabbitmq.client.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;


public class RMQMsgManager {

    private Channel channel;

    private Connection connection;

    private Address[] addresses;

    private String url;

    private ExecutorService executorService;

    private String exchangeName;

    private Map<String, String> RMQParams = new MsgContext().RMQParams;

    private Map<String, String> QueueNames = new HashMap<String, String>();

    public RMQMsgManager(String Name,
                         Address[] addresses,
                         String url) {
        this.exchangeName = Name;
        this.addresses = addresses;
        this.url = url;
        RMQParams.put(MsgContext.KEY_EXCHAGE_NAME, Name);
    }

    public RMQMsgManager(String Name,
                         ExecutorService executorService,
                         Address[] addresses,
                         String url) {
        this.exchangeName = Name;
        this.executorService = executorService;
        this.addresses = addresses;
        this.url = url;
        RMQParams.put(MsgContext.KEY_EXCHAGE_NAME, Name);
    }

    public void start() {
        setKeyQueueNames();
        setChannel();
    }

    private void setKeyQueueNames() {
        QueueNames.clear();
        QueueNames.keySet().add("OdometryMsg");
        QueueNames.keySet().add("ScanMsg");
        QueueNames.keySet().add("ParticleMsg");
        QueueNames.keySet().add("VelocityCmdMsg");
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

    private void BindQueue() {
        try {
            for (Map.Entry e : RMQParams.entrySet()) {
                if (e.getKey().equals(MsgContext.KEY_EXCHAGE_NAME))
                    continue;
                else if (e.getKey().equals(MsgContext.KEY_ROUTINGKEY_VELOCITY)) {
                    velConQueueName = channel.queueDeclare().getQueue();
                    channel.queueBind(velConQueueName, exchangeName, velRoutingKey);
                } else {

                    channel.queueBind(channel.queueDeclare().getQueue(), exchangeName, (String) e.getValue());
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
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
        } catch (IOException e) {
            System.out.println("Error closing the rabbit MQ connection" + e);
        }
    }
}
