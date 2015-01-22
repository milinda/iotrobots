package cgl.iotrobots.sim;

import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.utils.rabbitmq.Message;
import cgl.iotrobots.utils.rabbitmq.RabbitMQSender;

import java.util.HashMap;
import java.util.Map;

public class SimUtils {
    public static void sendControl(RabbitMQSender sender) {
        byte[] body = "start".getBytes();
        Map<String, Object> props = new HashMap<String, Object>();
        props.put("time", System.currentTimeMillis());
        props.put(TransportConstants.SENSOR_ID, System.currentTimeMillis());
        Message message = new Message(body, props);
        try {
            sender.send(message, "test.test.control");
            Thread.sleep(1000);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}