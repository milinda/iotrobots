package cgl.iotrobots.utils.rabbitmq;

import java.util.Map;

public interface MessageHandler {
    Map<String, String> getProperties();

    void onMessage(Message message);
}
