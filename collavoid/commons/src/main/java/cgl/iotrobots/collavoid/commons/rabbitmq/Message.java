package cgl.iotrobots.collavoid.commons.rabbitmq;

import java.util.HashMap;
import java.util.Map;

public class Message {
    byte[] body = null;

    private Map<String, Object> properties = new HashMap<String, Object>();

    public Message(byte[] body, Map<String, Object> properties) {
        this.body = body;
        this.properties = properties;
    }

    public byte[] getBody() {
        return body;
    }

    public Map<String, Object> getProperties() {
        return properties;
    }
}
