package cgl.iotrobots.collavoid.commons.storm;

import backtype.storm.Constants;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.rmqmsg.RMQContext;
import com.rabbitmq.client.Channel;
import io.latent.storm.rabbitmq.Declarator;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by hjh on 12/23/14.
 */
public class Methods_storm {
    public static boolean isTickTuple(Tuple tuple) {
        return tuple.getSourceComponent().equals(Constants.SYSTEM_COMPONENT_ID) && tuple.getSourceStreamId().equals(
                Constants.SYSTEM_TICK_STREAM_ID);
    }

    public static class StormDeclarator implements Declarator {
        private final String exchange;
        private final String queue;
        private final String routingKey;
        private final String exType;

        public StormDeclarator(String exchange, String queue) {
            this(exchange, queue, "", "");
        }

        public StormDeclarator(String exchange, String queue, String routingKey, String exType) {
            this.exchange = exchange;
            this.queue = queue;
            this.routingKey = routingKey;
            if (exType.equals(""))
                this.exType = "direct";
            else
                this.exType = exType;
        }

        @Override
        public void execute(Channel channel) {
            // you're given a RabbitMQ Channel so you're free to wire up your exchange/queue bindings as you see fit
            try {
                Map<String, Object> args = new HashMap<String, Object>();
                channel.exchangeDeclare(exchange, exType, true);
                if (!queue.equals("")) {
                    channel.queueDeclare(queue, false, false, false, args);
                    channel.queueBind(queue, exchange, routingKey);
                }
            } catch (IOException e) {
                throw new RuntimeException("Error executing rabbitmq declarations.", e);
            }
        }
    }

}
