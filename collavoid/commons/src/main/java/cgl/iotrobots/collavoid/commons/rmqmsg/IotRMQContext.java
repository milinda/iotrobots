package cgl.iotrobots.collavoid.commons.rmqmsg;

import cgl.iotcloud.core.transport.Channel;

import java.util.UUID;

public class IotRMQContext {
    public String EXCHANGE_NAME = null;
    public String ROUTING_KEY = null;
    public String QUEUE_NAME = null;

    public Channel CHANNEL = null;
    public String EXCHANGE_TYPE = Constant_msg.TYPE_EXCHANGE_TOPIC;
    public boolean DURABLE = false;

    public IotRMQContext(String MessageType, String sensorID) {
        this.EXCHANGE_NAME = MessageType;
        this.ROUTING_KEY = MessageType + "." + sensorID;
        UUID uuid = UUID.randomUUID();
        if (sensorID.equals("*"))
            this.QUEUE_NAME = MessageType + Constant_msg.RMQ_QUEUE_PREFIX;// + uuid;
        else
            this.QUEUE_NAME = MessageType + Constant_msg.RMQ_QUEUE_PREFIX + sensorID;// + "_" + uuid;// queue name should not be the same
    }

}

