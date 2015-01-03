package testTopology;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.generated.KillOptions;
import backtype.storm.generated.Nimbus;
import backtype.storm.generated.NotAliveException;
import backtype.storm.generated.StormTopology;
import backtype.storm.topology.IRichSpout;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.tuple.Fields;
import backtype.storm.utils.NimbusClient;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import cgl.iotrobots.collavoid.topology.*;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.ConnectionFactory;
import io.latent.storm.rabbitmq.Declarator;
import io.latent.storm.rabbitmq.RabbitMQSpout;
import io.latent.storm.rabbitmq.config.ConnectionConfig;
import io.latent.storm.rabbitmq.config.ConsumerConfig;
import io.latent.storm.rabbitmq.config.ConsumerConfigBuilder;
import org.apache.thrift7.TException;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class BuildTopology {
    private LocalCluster localCluster;
    private Config config;
    private int id;
    private String exchange;
    private String sensorid;

    private ConnectionConfig connectionConfig;
    private ConsumerConfig spoutConfig;
    private StormTopology stormTopology;
    private String toplogyName;

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
        toplogyName = Constant_msg.AGENT_ID_PREFIX + id;
        sensorid = toplogyName;

        exchange = Constant_msg.AGENT_ID_PREFIX + id + Constant_msg.RMQ_EXCHANGE_SUFFIX;
        builder = new TopologyBuilder();
    }

    public void submit() {
        stormTopology = builder.createTopology();
        localCluster.submitTopology(toplogyName, config, stormTopology);
    }

    public void shutdown() {
        odomSpout.close();
        particleSpout.close();
        Map conf = Utils.readStormConfig();
        Nimbus.Client client = NimbusClient.getConfiguredClient(conf).getClient();
        KillOptions killOpts = new KillOptions();
        try {
            client.killTopologyWithOpts(toplogyName, killOpts); //provide topology name
        } catch (NotAliveException e) {
            e.printStackTrace();
        } catch (TException e) {
            e.printStackTrace();
        }

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
                        exchange + Constant_msg.RMQ_QUEUE_PREFIX + Constant_msg.KEY_ODOMETRY + Constant_msg.RMQ_QUEUE_SUFFIX,
                        Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_ODOMETRY,
                        "")
        );

        scanSpout = new RabbitMQSpout(
                new Schemes.ScanScheme(),
                new StormDeclarator(
                        exchange,
                        exchange + Constant_msg.RMQ_QUEUE_PREFIX + Constant_msg.KEY_SCAN + Constant_msg.RMQ_QUEUE_SUFFIX,
                        Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_SCAN,
                        "")
        );

        particleSpout = new RabbitMQSpout(
                new Schemes.ParticleScheme(),
                new StormDeclarator(
                        exchange,
                        exchange + Constant_msg.RMQ_QUEUE_PREFIX + Constant_msg.KEY_PARTICLE_CLOUD + Constant_msg.RMQ_QUEUE_SUFFIX,
                        Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_PARTICLE_CLOUD,
                        "")
        );

        startGoalSpout = new RabbitMQSpout(
                new Schemes.startGoalScheme(),
                new StormDeclarator(
                        exchange,
                        exchange + Constant_msg.RMQ_QUEUE_PREFIX + Constant_msg.KEY_START_GOAL + Constant_msg.RMQ_QUEUE_SUFFIX,
                        Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_START_GOAL,
                        "")
        );
        // queue name can not be empty
        poseShareSpout = new RabbitMQSpout(
                new Schemes.PoseShareScheme(),
                new StormDeclarator(
                        Constant_msg.KEY_POSE_SHARE,
                        Constant_msg.KEY_POSE_SHARE,
                        "",
                        Constant_msg.TYPE_EXCHANGE_FANOUT)
        );
    }

    private void setAttachBolts() {
        builder.setSpout(Constant_storm.Components.TIMER_COMPONENT, new TimerSpout());
        
        builder.setSpout(Constant_storm.Components.ODOMETRY_COMPONENT + "Spout", odomSpout, 1)
                .addConfigurations(spoutConfig.asMap())
                .setMaxSpoutPending(200);
        builder.setBolt(Constant_storm.Components.ODOMETRY_COMPONENT,
                new AttachTimeID(sensorid, Constant_storm.FIELDS.ODOMETRY_FIELD), 1)
                .shuffleGrouping(Constant_storm.Components.ODOMETRY_COMPONENT + "Spout");//???? no sensor id at all

        builder.setSpout(Constant_storm.Components.SCAN_COMPONENT + "Spout", scanSpout, 1)
                .addConfigurations(spoutConfig.asMap())
                .setMaxSpoutPending(200);
        builder.setBolt(Constant_storm.Components.SCAN_COMPONENT,
                new AttachTimeID(sensorid, Constant_storm.FIELDS.SCAN_FIELD), 1)
                .shuffleGrouping(Constant_storm.Components.SCAN_COMPONENT + "Spout");

        builder.setSpout(Constant_storm.Components.POSE_ARRAY_COMPONENT + "Spout", particleSpout, 1)
                .addConfigurations(spoutConfig.asMap())
                .setMaxSpoutPending(200);
        builder.setBolt(Constant_storm.Components.POSE_ARRAY_COMPONENT,
                new AttachTimeID(sensorid, Constant_storm.FIELDS.POSE_ARRAY_FIELD), 1)
                .shuffleGrouping(Constant_storm.Components.POSE_ARRAY_COMPONENT + "Spout");

        builder.setSpout(Constant_storm.Components.START_GOAL_COMPONENT + "Spout", startGoalSpout, 1)
                .addConfigurations(spoutConfig.asMap())
                .setMaxSpoutPending(200);
        builder.setBolt(Constant_storm.Components.START_GOAL_COMPONENT,
                new AttachTimeID(sensorid, Constant_storm.FIELDS.START_GOAL_FIELD), 1)
                .shuffleGrouping(Constant_storm.Components.START_GOAL_COMPONENT + "Spout");

        builder.setSpout(Constant_storm.Components.POSE_SHARE_COMPONENT + "Spout", poseShareSpout, 1)
                .addConfigurations(spoutConfig.asMap())
                .setMaxSpoutPending(200);
        builder.setBolt(Constant_storm.Components.POSE_SHARE_COMPONENT,
                new AttachTimeID("", Constant_storm.FIELDS.POSE_SHARE_FIELD), 1)
                .shuffleGrouping(Constant_storm.Components.POSE_SHARE_COMPONENT + "Spout");
    }

    private static class StormDeclarator implements Declarator {
        private final String exchange;
        private final String queue;
        private final String routingKey;
        private final String exType;

        public StormDeclarator(String exchange, String queueName, String routingKey, String exType) {
            this.exchange = exchange;
            this.routingKey = routingKey;
            this.queue = queueName;
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
                channel.queueDeclare(queue, true, false, false, args);
                channel.queueBind(queue, exchange, routingKey);
                channel.queuePurge(queue);
            } catch (IOException e) {
                throw new RuntimeException("Error executing rabbitmq declarations.", e);
            }
        }
    }


    private void buildTopology() {
        builder.setBolt(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT, new GlobalPlannerBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.START_GOAL_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));

        builder.setBolt(Constant_storm.Components.LOCAL_PLANNER_COMPONENT, new LocalPlannerBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.ODOMETRY_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
//                .fieldsGrouping(Constant_storm.Components.VELOCITY_COMPUTE_COMPONENT,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .shuffleGrouping(Constant_storm.Components.TIMER_COMPONENT,
                        Constant_storm.Streams.CONTROLLER_TIMER_STREAM);

        builder.setBolt(Constant_storm.Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT, new VelCmdPubBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.LOCAL_PLANNER_COMPONENT,
                        Constant_storm.Streams.VELOCITY_COMMAND_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));

        builder.setBolt(Constant_storm.Components.GET_OBSTACLES_COMPONENT, new GetObstacleBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.SCAN_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .shuffleGrouping(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT);

        builder.setBolt(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT, new GetAllAgentsBolt(), 1)
                .shuffleGrouping(Constant_storm.Components.POSE_SHARE_COMPONENT);

        builder.setBolt(Constant_storm.Components.GET_MINKOWSKI_FOOTPRINT_COMPONENT, new GetMinkowskiFootprintBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.POSE_ARRAY_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.AGENT_COMPONENT,
                        Constant_storm.Streams.FOOTPRINT_OWN_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
//
        builder.setBolt(Constant_storm.Components.AGENT_STATE_COMPONENT, new AgentStateBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.ODOMETRY_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.GET_OBSTACLES_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .shuffleGrouping(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT)
                .fieldsGrouping(Constant_storm.Components.GET_MINKOWSKI_FOOTPRINT_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.LOCAL_PLANNER_COMPONENT,
                        Constant_storm.Streams.PREFERRED_VELOCITY_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .shuffleGrouping(Constant_storm.Components.TIMER_COMPONENT,
                        Constant_storm.Streams.PUBLISH_ME_TIMER_STREAM);
//
        builder.setBolt(Constant_storm.Components.AGENT_COMPONENT, new AgentBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.AGENT_STATE_COMPONENT,
                        Constant_storm.Streams.PUBLISHME_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.AGENT_STATE_COMPONENT,
                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
//
        builder.setBolt(Constant_storm.Components.ACC_CONSTRAINTS_COMPONENT, new AddAccelerationConstraintBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.AGENT_COMPONENT,
                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
        builder.setBolt(Constant_storm.Components.NH_CONSTRAINTS_COMPONENT, new AddNHConstraintsBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.AGENT_COMPONENT,
                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
        builder.setBolt(Constant_storm.Components.VO_AGENT_COMPONENT, new VOAgentBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.AGENT_COMPONENT,
                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
        builder.setBolt(Constant_storm.Components.VO_OBSTACLE_COMPONENT, new VOObstacleBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.AGENT_COMPONENT,
                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
//
//        builder.setBolt(Constant_storm.Components.VO_LINES_JOIN_COMPONENT,
//                new VOLinesJoinBolt(Constant_storm.FIELDS.JOIN_FIELDS), 1)
//                .fieldsGrouping(Constant_storm.Components.ACC_CONSTRAINTS_COMPONENT,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
//                .fieldsGrouping(Constant_storm.Components.NH_CONSTRAINTS_COMPONENT,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
//                .fieldsGrouping(Constant_storm.Components.VO_AGENT_COMPONENT,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
//                .fieldsGrouping(Constant_storm.Components.VO_OBSTACLE_COMPONENT,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
//
//        builder.setBolt(Constant_storm.Components.VELOCITY_COMPUTE_COMPONENT, new VelocityComputeBolt(), 1)
//                .fieldsGrouping(Constant_storm.Components.VO_LINES_JOIN_COMPONENT,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
    }

}
