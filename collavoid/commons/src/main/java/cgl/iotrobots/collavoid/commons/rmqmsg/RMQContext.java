package cgl.iotrobots.collavoid.commons.rmqmsg;

public class RMQContext {
    public String EXCHAGE_NAME;
    public String ROUTING_KEY;
    public String QUEUE_NAME;

    public RMQContext(String exchangeName, String msgName) {
        this.EXCHAGE_NAME = exchangeName;
        this.ROUTING_KEY = "RoutingKey_" + msgName;
        this.QUEUE_NAME = "Queue_" + msgName;
    }

}