package cgl.iotrobots.collavoid.commons.storm;

import backtype.storm.Config;
import backtype.storm.Constants;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.planners.*;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import com.esotericsoftware.kryo.Kryo;
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


    public static void addSerializers(Config config) {
        config.registerSerialization(Odometry_.class);
        config.registerSerialization(Header_.class);
        config.registerSerialization(Pose_.class);
        config.registerSerialization(Twist_.class);
        config.registerSerialization(PoseShareMsg_.class);
        config.registerSerialization(PoseArray_.class);
        config.registerSerialization(Vector3d_.class);
        config.registerSerialization(Vector4d_.class);
        config.registerSerialization(Vector2.class);
        config.registerSerialization(BaseConfig_.class);
        config.registerSerialization(PointCloud2_.class);
        config.registerSerialization(PoseStamped_.class);
        config.registerSerialization(Kryo.class);

        config.registerSerialization(Position.class);
        config.registerSerialization(Agent.class);
        config.registerSerialization(Neighbor.class);
        config.registerSerialization(VO.class);
        config.registerSerialization(Line.class);
        config.registerSerialization(LinePair.class);
        config.registerSerialization(Obstacle.class);
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
