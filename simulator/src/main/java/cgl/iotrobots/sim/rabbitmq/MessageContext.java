package cgl.iotrobots.sim.rabbitmq;

import java.util.HashMap;
import java.util.Map;

public class MessageContext {
    byte []body = null;

    private Map<String, Object> properties = new HashMap<String, Object>();

    public MessageContext(byte[] body, Map<String, Object> properties) {
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
