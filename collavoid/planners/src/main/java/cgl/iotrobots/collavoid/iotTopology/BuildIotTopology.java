package cgl.iotrobots.collavoid.iotTopology;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.generated.KillOptions;
import backtype.storm.generated.Nimbus;
import backtype.storm.generated.NotAliveException;
import backtype.storm.generated.StormTopology;
import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.IRichSpout;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.tuple.Fields;
import backtype.storm.utils.NimbusClient;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import org.apache.thrift7.TException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.HashMap;
import java.util.Map;

public class BuildIotTopology {
    private Logger logger = LoggerFactory.getLogger(BuildIotTopology.class);
    private LocalCluster localCluster=null;
    private Config config;

    private Map<String,IRichSpout> spoutMap=new HashMap<>();
    private Map<String,IRichBolt> boltMap=new HashMap<>();

    private StormTopology stormTopology;
    private String toplogyName;

    private TopologyBuilder builder = new TopologyBuilder();;

    public BuildIotTopology(final Config config){
        this(null,config,null);
    }
    public BuildIotTopology(final LocalCluster localCluster, Config config, String toplogyName) {
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
        this.toplogyName = toplogyName;
        StreamTopologyBuilder streamTopologyBuilder = new StreamTopologyBuilder();
        StreamComponents components = streamTopologyBuilder.buildComponents();
        spoutMap=components.getSpouts();
        boltMap=components.getBolts();


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
        buidSpouts();
        buildTopology();
    }

    private void buidSpouts() {

        for (Map.Entry<String,IRichSpout> e:spoutMap.entrySet()){
            builder.setSpout(e.getKey(),e.getValue(),1);
        }
        builder.setSpout(Constant_storm.Components.TIMER_SPOUT_COMPONENT, new TimerSpout());


    }


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

        // iot component
        builder.setBolt(Constant_storm.Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT,
                boltMap.get(Constant_storm.Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT), 1)
                .fieldsGrouping(Constant_storm.Components.LOCAL_PLANNER_COMPONENT,
                        Constant_storm.Streams.VELOCITY_COMMAND_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));

        builder.setBolt(Constant_storm.Components.GET_OBSTACLES_COMPONENT, new GetObstacleBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.SCAN_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .allGrouping(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT);

        // each topology has only one
        builder.setBolt(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT, new GetAllAgentsBolt(), 1)
                .shuffleGrouping(Constant_storm.Components.AGENT_STATE_COMPONENT,
                        Constant_storm.Streams.RESET_STREAM)
                .shuffleGrouping(Constant_storm.Components.AGENT_COMPONENT,
                        Constant_storm.Streams.PUBLISHME_STREAM);
//
//        builder.setBolt(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT, new GetAllAgentsBolt(), 1)
//                .shuffleGrouping(Constant_storm.Components.AGENT_STATE_COMPONENT,
//                        Constant_storm.Streams.RESET_STREAM)
//                .shuffleGrouping(Constant_storm.Components.POSE_SHARE_COMPONENT);
//
//        builder.setBolt(Constant_storm.Components.POSE_SHARE_PUB_COMPONENT,
//                boltMap.get(Constant_storm.Components.POSE_SHARE_PUB_COMPONENT),1)
//                .shuffleGrouping(Constant_storm.Components.AGENT_COMPONENT,
//                        Constant_storm.Streams.PUBLISHME_STREAM);

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
        builder.setBolt(Constant_storm.Components.AGENT_COMPONENT, new AgentBolt_(), 1)
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
}
