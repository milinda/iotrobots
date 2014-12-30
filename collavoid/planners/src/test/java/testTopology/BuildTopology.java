package testTopology;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.topology.IRichSpout;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.tuple.Fields;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.ConnectionFactory;
import io.latent.storm.rabbitmq.Declarator;
import io.latent.storm.rabbitmq.RabbitMQSpout;
import io.latent.storm.rabbitmq.config.ConnectionConfig;
import io.latent.storm.rabbitmq.config.ConsumerConfig;
import io.latent.storm.rabbitmq.config.ConsumerConfigBuilder;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class BuildTopology {
    private LocalCluster localCluster;
    private Config config;
    private int id;
    private String exchange;
    private String topName;
    private String sensorid;

    private ConnectionConfig connectionConfig;
    private ConsumerConfig spoutConfig;

    private IRichSpout odomSpout;
    private IRichSpout scanSpout;
    private IRichSpout particleSpout;
    private IRichSpout poseShareSpout;
    private IRichSpout startGoalSpout;
    private TopologyBuilder builder;

    public BuildTopology(LocalCluster localCluster, Config config, int ID) {
        this.localCluster = localCluster;
        this.config = config;
        this.id = ID;

        exchange = Constant_msg.AGENT_ID_PREFIX + id + Constant_msg.RMQ_EXCHANGE_SUFFIX;
        builder = new TopologyBuilder();
        topName = Constant_msg.AGENT_ID_PREFIX + id;
        sensorid = topName;
    }

    public void setTopology() {
        connectionConfig = new ConnectionConfig(
                "localhost",
                5672,
                "guest",
                "guest",
                ConnectionFactory.DEFAULT_VHOST,
                10); // host, port, username, password, virtualHost, heartBeat
        spoutConfig = new ConsumerConfigBuilder().connection(connectionConfig)
                .queue("")
                .prefetch(200)
                .requeueOnFail()
                .build();
        buidSpouts();
        setAttachBolts();
        buildTopology();
    }

    private void buidSpouts() {
        odomSpout = new RabbitMQSpout(
                new Schemes.OdometryScheme(),
                new StormDeclarator(
                        exchange,
                        Constant_msg.RMQ_QUEUE_PREFIX + Constant_msg.KEY_ODOMETRY,
                        Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_ODOMETRY,
                        "")
        );

        scanSpout = new RabbitMQSpout(
                new Schemes.ScanScheme(),
                new StormDeclarator(
                        exchange,
                        Constant_msg.RMQ_QUEUE_PREFIX + Constant_msg.KEY_SCAN,
                        Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_SCAN,
                        "")
        );

        particleSpout = new RabbitMQSpout(
                new Schemes.ParticleScheme(),
                new StormDeclarator(
                        exchange,
                        Constant_msg.RMQ_QUEUE_PREFIX + Constant_msg.KEY_PARTICLE_CLOUD,
                        Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_PARTICLE_CLOUD,
                        "")
        );

        poseShareSpout = new RabbitMQSpout(
                new Schemes.PoseShareScheme(),
                new StormDeclarator(
                        Constant_msg.KEY_POSE_SHARE,
                        "",
                        "",
                        Constant_msg.TYPE_EXCHANGE_FANOUT)
        );

    }

    private void setAttachBolts() {
        builder.setSpout(Constant_storm.Components.ODOMETRY_COMPONENT + "Spout", odomSpout, 1)
                .addConfigurations(spoutConfig.asMap())
                .setMaxSpoutPending(200);
        builder.setBolt(Constant_storm.Components.ODOMETRY_COMPONENT,
                new AttachTimeID(sensorid, Constant_storm.FIELDS.ODOMETRY_FIELD), 1)
                .fieldsGrouping(Constant_storm.Components.ODOMETRY_COMPONENT + "Spout",
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));

        builder.setSpout(Constant_storm.Components.SCAN_COMPONENT + "Spout", scanSpout, 1)
                .addConfigurations(spoutConfig.asMap())
                .setMaxSpoutPending(200);
        builder.setBolt(Constant_storm.Components.SCAN_COMPONENT,
                new AttachTimeID(sensorid, Constant_storm.FIELDS.SCAN_FIELD), 1)
                .fieldsGrouping(Constant_storm.Components.SCAN_COMPONENT + "Spout",
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));

        builder.setSpout(Constant_storm.Components.POSE_ARRAY_COMPONENT + "Spout", particleSpout, 1)
                .addConfigurations(spoutConfig.asMap())
                .setMaxSpoutPending(200);
        builder.setBolt(Constant_storm.Components.POSE_ARRAY_COMPONENT,
                new AttachTimeID(sensorid, Constant_storm.FIELDS.POSE_ARRAY_FIELD), 1)
                .fieldsGrouping(Constant_storm.Components.POSE_ARRAY_COMPONENT + "Spout",
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));

        builder.setSpout(Constant_storm.Components.POSE_SHARE_COMPONENT + "Spout", poseShareSpout, 1)
                .addConfigurations(spoutConfig.asMap())
                .setMaxSpoutPending(200);
        builder.setBolt(Constant_storm.Components.POSE_SHARE_COMPONENT,
                new AttachTimeID("", Constant_storm.FIELDS.POSE_SHARE_FIELD), 1)
                .shuffleGrouping(Constant_storm.Components.ODOMETRY_COMPONENT + "Spout");
    }

    private void buildTopology() {

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
                    channel.queueDeclare(queue, true, false, false, args);
                    channel.queueBind(queue, exchange, routingKey);
                }
            } catch (IOException e) {
                throw new RuntimeException("Error executing rabbitmq declarations.", e);
            }
        }
    }

}
