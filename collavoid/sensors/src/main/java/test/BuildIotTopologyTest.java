package test;

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

public class BuildIotTopologyTest {
    private Logger logger = LoggerFactory.getLogger(BuildIotTopologyTest.class);
    private LocalCluster localCluster = null;
    private Config config;

    private Map<String, IRichSpout> spoutMap = new HashMap<>();
    private Map<String, IRichBolt> boltMap = new HashMap<>();

    private StormTopology stormTopology;
    private String toplogyName;

    private TopologyBuilder builder = new TopologyBuilder();
    ;

    public BuildIotTopologyTest(final Config config) {
        this(null, config, null);
    }

    public BuildIotTopologyTest(final LocalCluster localCluster, Config config, String toplogyName) {
        if (localCluster != null) {
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
        spoutMap = components.getSpouts();
        boltMap = components.getBolts();


    }

    public void submit() {
        if (localCluster == null) {
            logger.error("Not in local mode, localCluster not received!!");
            return;
        }
        stormTopology = builder.createTopology();
        localCluster.submitTopology(toplogyName, config, stormTopology);
    }

    public StormTopology getStormTopology() {
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

        for (Map.Entry<String, IRichSpout> e : spoutMap.entrySet()) {
            builder.setSpout(e.getKey(), e.getValue(), 1);
        }

    }


    private void buildTopology() {
        builder.setSpout(Constant_storm.Components.TIMER_COMPONENT, new TimerSpoutTest(), 1);

        builder.setBolt(Constant_storm.Components.ODOMETRY_TRANSFORM_COMPONENT, new OdometryTransformBolt(), 1)
                .fieldsGrouping(Constant_storm.Components.ODOMETRY_COMPONENT,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD))
                .shuffleGrouping(Constant_storm.Components.TIMER_COMPONENT);

        // iot component
        builder.setBolt(Constant_storm.Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT,
                boltMap.get(Constant_storm.Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT), 1)
                .fieldsGrouping(Constant_storm.Components.ODOMETRY_TRANSFORM_COMPONENT,
                        Constant_storm.Streams.VELOCITY_COMMAND_STREAM,
                        new Fields(Constant_storm.FIELDS.SENSOR_ID_FIELD));

    }

}
