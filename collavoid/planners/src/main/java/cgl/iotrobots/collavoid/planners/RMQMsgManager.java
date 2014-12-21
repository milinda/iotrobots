package cgl.iotrobots.collavoid.planners;

import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import com.rabbitmq.client.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;


public class RMQMsgManager {

    private Channel channel, shareChannel;

    private boolean autoAck = false;

    private Address[] addresses;

    private String url;

    private String exchangeName;

    private Map<String, RMQContext> RMQContexts = new HashMap<String, RMQContext>();

    public RMQMsgManager(String Name,
                         Address[] addresses,
                         String url) {
        this.exchangeName = Name;
        this.addresses = addresses;
        this.url = url;

        RMQContexts.put(Constant.KEY_ODOMETRY, new RMQContext(exchangeName, Constant.KEY_ODOMETRY));
        RMQContexts.put(Constant.KEY_PARTICLE_CLOUD, new RMQContext(exchangeName, Constant.KEY_PARTICLE_CLOUD));
        RMQContexts.put(Constant.KEY_SCAN, new RMQContext(exchangeName, Constant.KEY_SCAN));
        RMQContexts.put(Constant.KEY_VELOCITY_CMD, new RMQContext(exchangeName, Constant.KEY_VELOCITY_CMD));

        RMQContexts.put(Constant.KEY_POSE_SHARE, new RMQContext(Constant.KEY_POSE_SHARE, ""));
        RMQContexts.get(Constant.KEY_POSE_SHARE).EXCHANGE_TYPE = Constant.TYPE_EXCHANGE_FANOUT;

    }

    public void start(Agent agent) {
        channel = Methods.getChannel(addresses, url, null, exchangeName, Constant.TYPE_EXCHANGE_DIRECT);
        shareChannel = Methods.getChannel(addresses, url, null, exchangeName, Constant.TYPE_EXCHANGE_FANOUT);
        bindQueue();
        bindCallBack(agent, RMQContexts);
    }

    private void bindQueue() {
        try {
            for (Map.Entry<String, RMQContext> e : RMQContexts.entrySet()) {
                if (e.getKey().equals(Constant.KEY_POSE_SHARE))
                    continue;
                e.getValue().QUEUE_NAME = channel.queueDeclare().getQueue();
                channel.queueBind(e.getValue().QUEUE_NAME, e.getValue().EXCHANGE_NAME, e.getValue().ROUTING_KEY);
            }
            RMQContexts.get(Constant.KEY_POSE_SHARE).QUEUE_NAME = shareChannel.queueDeclare().getQueue();
            shareChannel.queueBind(
                    RMQContexts.get(Constant.KEY_POSE_SHARE).QUEUE_NAME,
                    RMQContexts.get(Constant.KEY_POSE_SHARE).EXCHANGE_NAME,
                    RMQContexts.get(Constant.KEY_POSE_SHARE).ROUTING_KEY);
            
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void bindCallBack(Agent agent, Map<String, RMQContext> contexts) {
        try {
            String queueName = RMQContexts.get(Constant.KEY_ODOMETRY).QUEUE_NAME;
            String routingKey = RMQContexts.get(Constant.KEY_ODOMETRY).ROUTING_KEY;
            channel.basicConsume(queueName, autoAck, routingKey + "Tag",
                    new DefaultConsumer(channel) {
                        @Override
                        public void handleDelivery(String consumerTag,
                                                   Envelope envelope,
                                                   AMQP.BasicProperties properties,
                                                   byte[] body)
                                throws IOException {
                            long deliveryTag = envelope.getDeliveryTag();
                            Odometry_ odometry_ = JsonConverter.JSONToOdometry_(body);
                            velQueue.offer(velocity);
                            channel.basicAck(deliveryTag, false);
                        }
                    });
        } catch (IOException e) {
            String msg = "Error consuming the message";
            throw new RuntimeException(msg, e);
        } catch (Exception e) {
            String msg = "Error connecting to broker";
            throw new RuntimeException(msg, e);
        }

    }

    public Channel getChannel() {
        return channel;
    }

    public Channel getShareChannel() {
        return shareChannel;
    }

    public Map<String, RMQContext> getRMQContexts() {
        return RMQContexts;
    }

    public void stop() {
        try {
            if (channel != null) {
                channel.close();
            }
        } catch (IOException e) {
            System.out.println("Error closing the rabbit MQ connection" + e);
        }
    }
}
