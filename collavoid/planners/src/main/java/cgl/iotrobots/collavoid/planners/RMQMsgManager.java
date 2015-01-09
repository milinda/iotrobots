package cgl.iotrobots.collavoid.planners;

import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import com.rabbitmq.client.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class RMQMsgManager {

    private Channel channel, shareChannel;

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

        RMQContexts.put(Constant_msg.KEY_ODOMETRY, new RMQContext(exchangeName, Constant_msg.KEY_ODOMETRY));
        RMQContexts.put(Constant_msg.KEY_PARTICLE_CLOUD, new RMQContext(exchangeName, Constant_msg.KEY_PARTICLE_CLOUD));
        RMQContexts.put(Constant_msg.KEY_SCAN, new RMQContext(exchangeName, Constant_msg.KEY_SCAN));
        RMQContexts.put(Constant_msg.KEY_VELOCITY_CMD, new RMQContext(exchangeName, Constant_msg.KEY_VELOCITY_CMD));

        RMQContexts.put(Constant_msg.KEY_POSE_SHARE, new RMQContext(Constant_msg.KEY_POSE_SHARE, ""));
        RMQContexts.get(Constant_msg.KEY_POSE_SHARE).EXCHANGE_TYPE = Constant_msg.TYPE_EXCHANGE_FANOUT;

    }

    public void start(Agent agent) throws Exception {
        channel = Methods_RMQ.getChannel(
                addresses,
                url,
                null
        );
        channel.exchangeDeclare(exchangeName, Constant_msg.TYPE_EXCHANGE_DIRECT, true);
        shareChannel = Methods_RMQ.getChannel(
                addresses, url,
                null
        );
        channel.exchangeDeclare(RMQContexts.get(Constant_msg.KEY_POSE_SHARE).EXCHANGE_NAME,
                Constant_msg.TYPE_EXCHANGE_FANOUT, true);
        bindQueue();
        MsgCallBacks.bindCallBacks(agent, RMQContexts);
    }

    private void bindQueue() {
        try {
            for (Map.Entry<String, RMQContext> e : RMQContexts.entrySet()) {
                if (e.getKey().equals(Constant_msg.KEY_POSE_SHARE))
                    continue;
                e.getValue().CHANNEL = channel;
                e.getValue().QUEUE_NAME = channel.queueDeclare().getQueue();// may need to set queue size
                channel.queueBind(e.getValue().QUEUE_NAME, e.getValue().EXCHANGE_NAME, e.getValue().ROUTING_KEY);
            }
            RMQContexts.get(Constant_msg.KEY_POSE_SHARE).CHANNEL = shareChannel;
            RMQContexts.get(Constant_msg.KEY_POSE_SHARE).QUEUE_NAME = shareChannel.queueDeclare().getQueue();
            shareChannel.queueBind(
                    RMQContexts.get(Constant_msg.KEY_POSE_SHARE).QUEUE_NAME,
                    RMQContexts.get(Constant_msg.KEY_POSE_SHARE).EXCHANGE_NAME,
                    RMQContexts.get(Constant_msg.KEY_POSE_SHARE).ROUTING_KEY);
            
        } catch (IOException e) {
            e.printStackTrace();
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
            if (shareChannel != null) {
                shareChannel.close();
            }
        } catch (IOException e) {
            System.out.println("Error closing the rabbit MQ connection" + e);
        }
    }
}
