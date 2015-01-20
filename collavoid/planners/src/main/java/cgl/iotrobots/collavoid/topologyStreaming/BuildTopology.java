package cgl.iotrobots.collavoid.topologyStreaming;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.generated.KillOptions;
import backtype.storm.generated.Nimbus;
import backtype.storm.generated.NotAliveException;
import backtype.storm.generated.StormTopology;
import backtype.storm.spout.Scheme;
import backtype.storm.topology.IRichSpout;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.tuple.Fields;
import backtype.storm.utils.NimbusClient;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.rmqmsg.Contexts;
import cgl.iotrobots.collavoid.commons.rmqmsg.RMQContext;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.ConnectionFactory;
import io.latent.storm.rabbitmq.Declarator;
import io.latent.storm.rabbitmq.RabbitMQSpout;
import io.latent.storm.rabbitmq.config.ConnectionConfig;
import io.latent.storm.rabbitmq.config.ConsumerConfig;
import io.latent.storm.rabbitmq.config.ConsumerConfigBuilder;
import org.apache.thrift7.TException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class BuildTopology {
    private Logger logger = LoggerFactory.getLogger(BuildTopology.class);
    private LocalCluster localCluster=null;
    private Config config;
    private int id;
    private Map<String, RMQContext> RMQContexts;
    private Map<String, String> KeyToComponentMap = new Constant_storm.KeyToComponentMap().map;
    private Map<String, Scheme> schemeMap = new Schemes().schemeMap;
    private String sensorid;

    private ConnectionConfig connectionConfig;
    private ConsumerConfig spoutConfig;
    private StormTopology stormTopology;
    private String toplogyName;

    private Map<String, RabbitMQSpout> spoutMap = new HashMap<String, RabbitMQSpout>();
    private Map<String, StormDeclarator> declaratorMap = new HashMap<String, StormDeclarator>();

    private IRichSpout odomSpout;
    private IRichSpout scanSpout;
    private IRichSpout particleSpout;
    private IRichSpout poseShareSpout;
    private IRichSpout startGoalSpout;
    private TopologyBuilder builder = new TopologyBuilder();;

    public BuildTopology(final Config config){
        this(null,config,null);
    }
    public BuildTopology(final LocalCluster localCluster, Config config, String toplogyName) {
        if (localCluster!=null){
            this.localCluster = localCluster;
            Runtime.getRuntime().addShutdownHook(new Thread() {
                public void run() {
                    try {
                        localCluster.shutdown();
                    } catch (Exception e) {
                    }
                }
            });
        }
        this.config = config;

//        this.id = ID;
        this.toplogyName = toplogyName;
//        sensorid = toplogyName;

        RMQContexts = new Contexts("*").getRMQContexts();
        RMQContexts.remove(Constant_msg.KEY_VELOCITY_CMD);
        RMQContexts.put(Constant_msg.KEY_POSE_SHARE, new RMQContext(Constant_msg.KEY_POSE_SHARE, "*"));
        RMQContexts.get(Constant_msg.KEY_POSE_SHARE).EXCHANGE_TYPE = Constant_msg.TYPE_EXCHANGE_FANOUT;

    }

    public void submit() {
        if (localCluster==null){
            logger.error("Not in local mode, localCluster not received!!");
            return;
        }
        stormTopology = builder.createTopology();
        localCluster.submitTopology(toplogyName, config, stormTopology);
    }

    public StormTopology getStormTopology(){
        setTopology();
        stormTopology = builder.createTopology();
        return stormTopology;
    }

    public void shutdown() {
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
                Constant_msg.RMQ_IP,
                Constant_msg.RMQ_PORT,
                "guest",
                "guest",
                ConnectionFactory.DEFAULT_VHOST,
                10); // host, port, username, password, virtualHost, heartBeat
        spoutConfig = new ConsumerConfigBuilder().connection(connectionConfig)
                .queue("")
                .prefetch(10)
                .requeueOnFail()
                .build();

        buidSpouts();
//        setAttachBolts();
        buildTopology();
    }

    private void buidSpouts() {

        for (Map.Entry<String, RMQContext> e : RMQContexts.entrySet()) {
            Scheme scheme = schemeMap.get(e.getKey());
            StormDeclarator declarator = new StormDeclarator(e.getValue());
            RabbitMQSpout spout = new RabbitMQSpout(scheme, declarator);
            declaratorMap.put(e.getKey(), declarator);
            spoutMap.put(e.getKey(), spout);
            builder.setSpout(KeyToComponentMap.get(e.getKey()),
                    spoutMap.get(e.getKey()))
                    .addConfigurations(spoutConfig.asMap())
                    .setMaxSpoutPending(10);
            ;
        }
        builder.setSpout(Constant_storm.Components.TIMER_SPOUT_COMPONENT, new TimerSpout());

//        odomSpout = new RabbitMQSpout(
//                new Schemes.OdometryScheme(),
//                new StormDeclarator(
//                        exchange,
//                        exchange + Constant_msg.RMQ_QUEUE_PREFIX + Constant_msg.KEY_ODOMETRY + Constant_msg.RMQ_QUEUE_SUFFIX,
//                        Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_ODOMETRY,
//                        "")
//        );
//
//        scanSpout = new RabbitMQSpout(
//                new Schemes.ScanScheme(),
//                new StormDeclarator(
//                        exchange,
//                        exchange + Constant_msg.RMQ_QUEUE_PREFIX + Constant_msg.KEY_SCAN + Constant_msg.RMQ_QUEUE_SUFFIX,
//                        Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_SCAN,
//                        "")
//        );
//
//        particleSpout = new RabbitMQSpout(
//                new Schemes.ParticleScheme(),
//                new StormDeclarator(
//                        exchange,
//                        exchange + Constant_msg.RMQ_QUEUE_PREFIX + Constant_msg.KEY_PARTICLE_CLOUD + Constant_msg.RMQ_QUEUE_SUFFIX,
//                        Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_PARTICLE_CLOUD,
//                        "")
//        );
//
//        startGoalSpout = new RabbitMQSpout(
//                new Schemes.startGoalScheme(),
//                new StormDeclarator(
//                        exchange,
//                        exchange + Constant_msg.RMQ_QUEUE_PREFIX + Constant_msg.KEY_START_GOAL + Constant_msg.RMQ_QUEUE_SUFFIX,
//                        Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_START_GOAL,
//                        "")
//        );
//        // queue name can not be empty
//        poseShareSpout = new RabbitMQSpout(
//                new Schemes.PoseShareScheme(),
//                new StormDeclarator(
//                        Constant_msg.KEY_POSE_SHARE,
//                        Constant_msg.KEY_POSE_SHARE,
//                        "",
//                        Constant_msg.TYPE_EXCHANGE_FANOUT)
//        );
    }

//    private void setAttachBolts() {
//        builder.setSpout(Constant_storm.Components.TIMER_SPOUT_COMPONENT, new TimerSpout());
//
//        builder.setSpout(Constant_storm.Components.ODOMETRY_COMPONENT + "Spout", odomSpout, 1)
//                .addConfigurations(spoutConfig.asMap())
//                .setMaxSpoutPending(200);
//        builder.setBolt(Constant_storm.Components.ODOMETRY_COMPONENT,
//                new AttachTimeID(sensorid, Constant_storm.FIELDS.ODOMETRY_FIELD), 1)
//                .shuffleGrouping(Constant_storm.Components.ODOMETRY_COMPONENT + "Spout");//???? no sensor id at all
//
//        builder.setSpout(Constant_storm.Components.SCAN_COMPONENT + "Spout", scanSpout, 1)
//                .addConfigurations(spoutConfig.asMap())
//                .setMaxSpoutPending(200);
//        builder.setBolt(Constant_storm.Components.SCAN_COMPONENT,
//                new AttachTimeID(sensorid, Constant_storm.FIELDS.SCAN_FIELD), 1)
//                .shuffleGrouping(Constant_storm.Components.SCAN_COMPONENT + "Spout");
//
//        builder.setSpout(Constant_storm.Components.POSE_ARRAY_COMPONENT + "Spout", particleSpout, 1)
//                .addConfigurations(spoutConfig.asMap())
//                .setMaxSpoutPending(200);
//        builder.setBolt(Constant_storm.Components.POSE_ARRAY_COMPONENT,
//                new AttachTimeID(sensorid, Constant_storm.FIELDS.POSE_ARRAY_FIELD), 1)
//                .shuffleGrouping(Constant_storm.Components.POSE_ARRAY_COMPONENT + "Spout");
//
//        builder.setSpout(Constant_storm.Components.BASE_CONFIG_COMPONENT + "Spout", startGoalSpout, 1)
//                .addConfigurations(spoutConfig.asMap())
//                .setMaxSpoutPending(200);
//        builder.setBolt(Constant_storm.Components.BASE_CONFIG_COMPONENT,
//                new AttachTimeID(sensorid, Constant_storm.FIELDS.START_GOAL_FIELD), 1)
//                .shuffleGrouping(Constant_storm.Components.BASE_CONFIG_COMPONENT + "Spout");
//
//        builder.setSpout(Constant_storm.Components.POSE_SHARE_COMPONENT + "Spout", poseShareSpout, 1)
//                .addConfigurations(spoutConfig.asMap())
//                .setMaxSpoutPending(200);
//        builder.setBolt(Constant_storm.Components.POSE_SHARE_COMPONENT,
//                new AttachTimeID("", Constant_storm.FIELDS.POSE_SHARE_FIELD), 1)
//                .shuffleGrouping(Constant_storm.Components.POSE_SHARE_COMPONENT + "Spout");
//    }


    private void buildTopology() {
        builder.setBolt(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT, new GlobalPlannerBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.BASE_CONFIG_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));

        builder.setBolt(Constant_storm.Components.ODOMETRY_TRANSFORM_COMPONENT, new OdometryTransformBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.ODOMETRY_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));

        builder.setBolt(Constant_storm.Components.TIMER_COMPONENT, new TimerBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.BASE_CONFIG_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .allGrouping(Constant_storm.Components.TIMER_SPOUT_COMPONENT);

        builder.setBolt(Constant_storm.Components.LOCAL_PLANNER_COMPONENT, new LocalPlannerBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.ODOMETRY_TRANSFORM_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.VELOCITY_COMPUTE_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.TIMER_COMPONENT,
                        Constant_storm.Streams.CONTROLLER_TIMER_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));

        builder.setBolt(Constant_storm.Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT, new VelCmdPubBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.LOCAL_PLANNER_COMPONENT,
                        Constant_storm.Streams.VELOCITY_COMMAND_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));

        builder.setBolt(Constant_storm.Components.GET_OBSTACLES_COMPONENT, new GetObstacleBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.SCAN_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .allGrouping(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT);

        // each topology has only one
        builder.setBolt(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT, new GetAllAgentsBolt(), 2)
                .shuffleGrouping(Constant_storm.Components.AGENT_STATE_COMPONENT,
                        Constant_storm.Streams.RESET_STREAM)
                .shuffleGrouping(Constant_storm.Components.POSE_SHARE_COMPONENT);

        builder.setBolt(Constant_storm.Components.GET_MINKOWSKI_FOOTPRINT_COMPONENT, new GetMinkowskiFootprintBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.POSE_ARRAY_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.AGENT_COMPONENT,
                        Constant_storm.Streams.FOOTPRINT_OWN_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
//
        builder.setBolt(Constant_storm.Components.AGENT_STATE_COMPONENT, new AgentStateBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.ODOMETRY_TRANSFORM_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.GET_OBSTACLES_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.GET_MINKOWSKI_FOOTPRINT_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.LOCAL_PLANNER_COMPONENT,
                        Constant_storm.Streams.PREFERRED_VELOCITY_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.LOCAL_PLANNER_COMPONENT,
                        Constant_storm.Streams.RESET_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .allGrouping(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT)
                .fieldsGrouping(Constant_storm.Components.TIMER_COMPONENT,
                        Constant_storm.Streams.PUBLISH_ME_TIMER_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
//
        builder.setBolt(Constant_storm.Components.AGENT_COMPONENT, new AgentBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.AGENT_STATE_COMPONENT,
                        Constant_storm.Streams.PUBLISHME_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .fieldsGrouping(Constant_storm.Components.AGENT_STATE_COMPONENT,
                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
        //original paralleled topology
//        builder.setBolt(Constant_storm.Components.ACC_CONSTRAINTS_COMPONENT, new AddAccelerationConstraintBolt(), 1)
//                .fieldsGrouping(Constant_storm.Components.AGENT_COMPONENT,
//                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
//        builder.setBolt(Constant_storm.Components.NH_CONSTRAINTS_COMPONENT, new AddNHConstraintsBolt(), 1)
//                .fieldsGrouping(Constant_storm.Components.AGENT_COMPONENT,
//                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
//        builder.setBolt(Constant_storm.Components.VO_AGENT_COMPONENT, new VOAgentBolt(), 1)
//                .fieldsGrouping(Constant_storm.Components.AGENT_COMPONENT,
//                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
//        builder.setBolt(Constant_storm.Components.VO_OBSTACLE_COMPONENT, new VOObstacleBolt(), 1)
//                .fieldsGrouping(Constant_storm.Components.AGENT_COMPONENT,
//                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
//
//        builder.setBolt(Constant_storm.Components.VO_LINES_JOIN_COMPONENT,
//                new VOLinesJoinBolt(Constant_storm.FIELDS.JOIN_FIELDS), 2)
//                .fieldsGrouping(Constant_storm.Components.ACC_CONSTRAINTS_COMPONENT,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
//                .fieldsGrouping(Constant_storm.Components.NH_CONSTRAINTS_COMPONENT,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
//                .fieldsGrouping(Constant_storm.Components.VO_AGENT_COMPONENT,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
//                .fieldsGrouping(Constant_storm.Components.VO_OBSTACLE_COMPONENT,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));
//
//        builder.setBolt(Constant_storm.Components.VELOCITY_COMPUTE_COMPONENT, new VelocityComputeBolt(), 2)
//                .fieldsGrouping(Constant_storm.Components.VO_LINES_JOIN_COMPONENT,
//                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));

        // merge all parallel components into one
        builder.setBolt(Constant_storm.Components.VO_LINES_COMPUTE_COMPONENT, new VOLinesComputeBolt_(), 4)
                .shuffleGrouping(Constant_storm.Components.AGENT_COMPONENT, Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM);
        builder.setBolt(Constant_storm.Components.VELOCITY_COMPUTE_COMPONENT, new VelocityComputeBolt(), 2)
                .shuffleGrouping(Constant_storm.Components.VO_LINES_COMPUTE_COMPONENT);
    }

    private static class StormDeclarator implements Declarator {
        private final String exchange;
        private final String queue;
        private final String routingKey;
        private final String exType;

        public StormDeclarator(RMQContext context) {
            this.exchange = context.EXCHANGE_NAME;
            this.routingKey = context.ROUTING_KEY;
            this.exType = context.EXCHANGE_TYPE;
            this.queue = context.QUEUE_NAME;
        }

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
                args.put("x-max-length", 3);
                channel.exchangeDeclare(exchange, exType, false);
                channel.queueDeclare(queue, false, false, true, args);
                channel.queueBind(queue, exchange, routingKey);
                channel.queuePurge(queue);
            } catch (IOException e) {
                throw new RuntimeException("Error executing rabbitmq declarations.", e);
            }
        }

    }

}
