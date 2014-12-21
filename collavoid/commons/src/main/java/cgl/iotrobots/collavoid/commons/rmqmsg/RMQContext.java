package cgl.iotrobots.collavoid.commons.rmqmsg;

public class RMQContext {
    public String EXCHANGE_NAME = "";
    public String ROUTING_KEY = "";
    public String QUEUE_NAME = "";
    public String EXCHANGE_TYPE = Constant.TYPE_EXCHANGE_DIRECT;

    public RMQContext(String exchangeName, String msgName) {
        if (exchangeName != null)
            this.EXCHANGE_NAME = exchangeName;
        if (msgName != null) {
        this.ROUTING_KEY = "RoutingKey_" + msgName;
            this.QUEUE_NAME = "Queue_" + msgName;
        }
    }

}