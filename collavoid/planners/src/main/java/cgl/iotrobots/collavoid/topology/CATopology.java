package cgl.iotrobots.collavoid.topology;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.generated.StormTopology;
import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.IRichSpout;
import backtype.storm.topology.TopologyBuilder;
import cgl.iotrobots.collavoid.commons.iotcloud.ModStreamGrouping;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.Map;

public class CATopology {
    private Logger logger = LoggerFactory.getLogger(CATopology.class);
    private LocalCluster localCluster = null;
    private Config config;
    private Map<String, IRichSpout> spoutMap = new HashMap<String, IRichSpout>();
    private Map<String, IRichBolt> boltMap = new HashMap<String, IRichBolt>();
    private StormTopology stormTopology;
    private String topologyName;
    private TopologyBuilder builder = new TopologyBuilder();
    private int paraCompute = 3;
    private int paraState = 1;


    public CATopology(final Config config) {
        this(null, config, null);
    }

    public CATopology(final LocalCluster localCluster, Config config, String topologyName) {
        if (localCluster != null) {
            this.localCluster = localCluster;
        }
        this.config = config;
        this.topologyName = topologyName;
        StreamTopologyBuilder streamTopologyBuilder = new StreamTopologyBuilder();
        StreamComponents components = streamTopologyBuilder.buildComponents();
        spoutMap = components.getSpouts();
        boltMap = components.getBolts();
    }

    //  for local mode
    public void submit() {
        if (localCluster == null) {
            logger.error("Not in local mode, localCluster not set!!");
            return;
        }

        localCluster.submitTopology(topologyName, config, stormTopology);
    }

    public void shutdown() {
        if (localCluster != null) {
            localCluster.killTopology(topologyName);
            localCluster.shutdown();
        }
    }

    //  for remote mode
    public StormTopology buildCATopology(Integer paraCompute, Integer paraState) {
        if (paraCompute != null)
            this.paraCompute = paraCompute;
        if (paraState != null)
            this.paraState = paraState;
        buildTopology();
        return stormTopology;
    }

    private void buildTopology() {
        //spout
        for (Map.Entry<String, IRichSpout> e : spoutMap.entrySet()) {
            builder.setSpout(e.getKey(), e.getValue(), 1);
        }
        builder.setSpout(Constant_storm.Components.TIMER_SPOUT_COMPONENT, new TimerSpout());

        //bolt
        // manually ack tuple
        builder.setBolt(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT, new GlobalPlannerBolt(), 1)
                .allGrouping(Constant_storm.Components.TIMER_SPOUT_COMPONENT,
                        Constant_storm.Streams.TIMEOUT_STREAM)
                .customGrouping(Constant_storm.Components.COMMAND_SPOUT_COMPONENT,
                        new ModStreamGrouping())
                .customGrouping(Constant_storm.Components.AGENT_STATE_COMPONENT,
                        Constant_storm.Streams.ACK_STREAM,
                        new ModStreamGrouping())
                .customGrouping(Constant_storm.Components.DISPATCHER_COMPONENT,
                        Constant_storm.Streams.ACK_STREAM,
                        new ModStreamGrouping())
                .customGrouping(Constant_storm.Components.VELOCITY_COMPUTE_COMPONENT,
                        Constant_storm.Streams.ACK_STREAM,
                        new ModStreamGrouping());

        builder.setBolt(Constant_storm.Components.DISPATCHER_COMPONENT, new DispatcherBolt(), 1)
                .allGrouping(Constant_storm.Components.TIMER_SPOUT_COMPONENT,
                        Constant_storm.Streams.CTLPUB_TIMEER_STREAM)
                .customGrouping(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT,
                        Constant_storm.Streams.CTL_PUB_TIME_STREAM,
                        new ModStreamGrouping())
                .customGrouping(Constant_storm.Components.AGENT_STATE_COMPONENT,
                        Constant_storm.Streams.PUBLISHME_STREAM,
                        new ModStreamGrouping())
                .customGrouping(Constant_storm.Components.VELOCITY_COMPUTE_COMPONENT,
                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                        new ModStreamGrouping());

        builder.setBolt(Constant_storm.Components.VELOCITY_COMPUTE_COMPONENT, new VelocityComputeBolt(), paraCompute)
                .customGrouping(Constant_storm.Components.AGENT_STATE_COMPONENT,
                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                        new ModStreamGrouping())
                .customGrouping(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT,
                        Constant_storm.Streams.AGENT_STREAM,
                        new ModStreamGrouping());

        builder.setBolt(Constant_storm.Components.AGENT_STATE_COMPONENT, new AgentStateBolt(), paraState)
                .allGrouping(Constant_storm.Components.POSE_SHARE_COMPONENT)
                .customGrouping(Constant_storm.Components.ODOMETRY_SPOUT_COMPONENT,
                        new ModStreamGrouping())
                .customGrouping(Constant_storm.Components.POSE_ARRAY_COMPONENT,
                        new ModStreamGrouping())
                .customGrouping(Constant_storm.Components.SCAN_COMPONENT,
                        new ModStreamGrouping())
                .customGrouping(Constant_storm.Components.DISPATCHER_COMPONENT,
                        Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                        new ModStreamGrouping())
                .customGrouping(Constant_storm.Components.DISPATCHER_COMPONENT,
                        Constant_storm.Streams.PUBLISHME_STREAM,
                        new ModStreamGrouping())
                .customGrouping(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT,
                        Constant_storm.Streams.POSE_SHARE_MSG_STREAM,
                        new ModStreamGrouping());

        // iot component
        builder.setBolt(Constant_storm.Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT,
                boltMap.get(Constant_storm.Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT), 1)
                .shuffleGrouping(Constant_storm.Components.VELOCITY_COMPUTE_COMPONENT,
                        Constant_storm.Streams.VELOCITY_COMMAND_STREAM);

        builder.setBolt(Constant_storm.Components.POSE_SHARE_PUB_COMPONENT,
                boltMap.get(Constant_storm.Components.POSE_SHARE_PUB_COMPONENT), 1)
                .shuffleGrouping(Constant_storm.Components.AGENT_STATE_COMPONENT,
                        Constant_storm.Streams.PUBLISHME_PUB_STREAM);

        stormTopology = builder.createTopology();
    }

}
