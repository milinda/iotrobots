package cgl.iotrobots.collavoid.commons.rmqmsg;

public class IotRMQContext {
    public String EXCHANGE_NAME = null;
    public String ROUTING_KEY = null;
    public String QUEUE_NAME = null;

    public String CHANNEL = null;

    public IotRMQContext(String MessageType) {
        if (MessageType.indexOf("pose_share") >= 0) {
            this.EXCHANGE_NAME = "pose_share";
            this.ROUTING_KEY = "pose_share";
        } else {
            this.EXCHANGE_NAME = MessageType;
            this.ROUTING_KEY = MessageType;
        }
        this.QUEUE_NAME = MessageType;
        this.CHANNEL = MessageType.substring(0, MessageType.lastIndexOf("_"));
    }

}

