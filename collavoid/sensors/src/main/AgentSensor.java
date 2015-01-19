import cgl.iotcloud.core.AbstractConfigurator;
import cgl.iotcloud.core.AbstractSensor;
import cgl.iotcloud.core.Configurator;
import cgl.iotcloud.core.SensorContext;
import cgl.iotcloud.core.sensorsite.SiteContext;
import cgl.iotcloud.core.transport.Channel;
import cgl.iotcloud.core.transport.Direction;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.rmqmsg.RMQContext;

import java.util.HashMap;
import java.util.Map;

public class AgentSensor extends AbstractSensor {
    public static final String BROKER_URL = "broker_url";
    public static final String SENSOR_ID = "robot0";
    public static final String CHANNEL_NAME = "odometry_sender";
    public static final String TRANSPORT = "rabbitmq";

    @Override
    public Configurator getConfigurator(Map map) {
        return new AgentSensorConfigurator();
    }

    @Override
    public void open(SensorContext sensorContext) {
        final Channel odomSendChannel = sensorContext.getChannel(TRANSPORT, CHANNEL_NAME);

    }

    @Override
    public void close() {
        super.close();
    }

    private class AgentSensorConfigurator extends AbstractConfigurator {
        @Override
        public SensorContext configure(SiteContext siteContext, Map conf) {
            String brokerUrl = (String) conf.get(BROKER_URL);

            SensorContext context = new SensorContext(SENSOR_ID);
            context.addProperty(BROKER_URL, brokerUrl);

            RMQContext rmqContext = new RMQContext(Constant_msg.KEY_ODOMETRY, SENSOR_ID);
            Map sendProps = new HashMap();
            sendProps.put("exchange", rmqContext.EXCHANGE_NAME);
            sendProps.put("routingKey", rmqContext.ROUTING_KEY);
            sendProps.put("queueName", rmqContext.QUEUE_NAME);
            Channel sendChannel = createChannel(CHANNEL_NAME, sendProps, Direction.OUT, 1024);

            context.addChannel(TRANSPORT, sendChannel);

            return context;
        }
    }
}
