package cgl.iotrobots.slam.streaming;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.app.Position;
import cgl.iotrobots.slam.core.grid.Array2D;
import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.grid.HierarchicalArray2D;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.gridfastsalm.TNode;
import cgl.iotrobots.slam.core.scanmatcher.PointAccumulator;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.sensor.RangeSensor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.core.utils.DoublePoint;
import cgl.iotrobots.slam.core.utils.IntPoint;
import cgl.iotrobots.slam.streaming.msgs.*;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import cgl.sensorstream.core.rabbitmq.DefaultRabbitMQMessageBuilder;
import com.ss.commons.*;
import com.ss.rabbitmq.ErrorReporter;
import com.ss.rabbitmq.RabbitMQSpout;
import com.ss.rabbitmq.bolt.RabbitMQBolt;
import org.apache.commons.cli.BasicParser;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.Options;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.Map;

public class SLAMTopology {
    private static Logger LOG = LoggerFactory.getLogger(SLAMTopology.class);

    public static void main(String[] args) throws Exception {
        TopologyBuilder builder = new TopologyBuilder();

        Options options = new Options();
        options.addOption(Constants.ARGS_NAME, true, "Name of the topology");
        options.addOption(Constants.ARGS_LOCAL, false, "Weather we want run locally");
        options.addOption(Constants.ARGS_DS_MODE, true, "The distributed mode, specify 0, 1, 2, 3 etc");

        CommandLineParser commandLineParser = new BasicParser();
        CommandLine cmd = commandLineParser.parse(options, args);
        String name = cmd.getOptionValue(Constants.ARGS_NAME);
        boolean local = cmd.hasOption(Constants.ARGS_LOCAL);
        String dsModeValue = cmd.getOptionValue(Constants.ARGS_DS_MODE);
        int dsMode = Integer.parseInt(dsModeValue);

        StreamTopologyBuilder streamTopologyBuilder;
        if (dsMode == 0) {
            streamTopologyBuilder = new StreamTopologyBuilder();
            buildTestTopology(builder, streamTopologyBuilder);
        }

        Config conf = new Config();
        conf.setDebug(false);
        // we are not going to track individual messages, message loss is inherent in the decoder
        // also we cannot replay message because of the decoder
        conf.put(Config.TOPOLOGY_ACKER_EXECUTORS, 0);

        // add the serializers
        addSerializers(conf);

        // we are going to deploy on a real cluster
        if (!local) {
            conf.setNumWorkers(6);
            StormSubmitter.submitTopology(name, conf, builder.createTopology());
        } else {
            // deploy on a local cluster
            conf.setMaxTaskParallelism(5);
            LocalCluster cluster = new LocalCluster();
            cluster.submitTopology("drone", conf, builder.createTopology());
            Thread.sleep(1000000);
            cluster.shutdown();
        }
    }

    private static void buildAllInOneTopology(TopologyBuilder builder, StreamTopologyBuilder streamTopologyBuilder) {
        StreamComponents components = streamTopologyBuilder.buildComponents();
    }

    private static void buildTestTopology(TopologyBuilder builder, StreamTopologyBuilder streamTopologyBuilder) {

        // first create a rabbitmq Spout
        ErrorReporter reporter = new ErrorReporter() {
            @Override
            public void reportError(Throwable throwable) {
                LOG.error("error occured", throwable);
            }
        };
        RabbitMQSpout spout = new RabbitMQSpout(new RabbitMQStaticSpoutConfigurator(), reporter);
        RabbitMQBolt sendBolt = new RabbitMQBolt(new RabbitMQStaticBoltConfigurator(), reporter);

        ScanMatchBolt scanMatchBolt = new ScanMatchBolt();
        ReSamplingBolt reSamplingBolt = new ReSamplingBolt();
        MapBuildingBolt mapBuildingBolt = new MapBuildingBolt();

        builder.setSpout(Constants.Topology.RECEIVE_SPOUT, spout, 1);
        builder.setBolt(Constants.Topology.SCAN_MATCH_BOLT, scanMatchBolt, 2).allGrouping(Constants.Topology.RECEIVE_SPOUT);
        builder.setBolt(Constants.Topology.RE_SAMPLE_BOLT, reSamplingBolt, 1).shuffleGrouping(Constants.Topology.SCAN_MATCH_BOLT, Constants.Fields.PARTICLE_STREAM);
        builder.setBolt(Constants.Topology.MAP_BOLT, mapBuildingBolt, 1).shuffleGrouping(Constants.Topology.SCAN_MATCH_BOLT, Constants.Fields.MAP_STREAM);
        builder.setBolt(Constants.Topology.SEND_BOLD, sendBolt, 1).shuffleGrouping(Constants.Topology.MAP_BOLT);
    }

    private static void addSerializers(Config config) {
        config.registerSerialization(DoublePoint.class);
        config.registerSerialization(IntPoint.class);
        config.registerSerialization(Particle.class);
        config.registerSerialization(GMap.class);
        config.registerSerialization(Array2D.class);
        config.registerSerialization(HierarchicalArray2D.class);
        config.registerSerialization(TNode.class);
        config.registerSerialization(DoubleOrientedPoint.class);
        config.registerSerialization(Particle.class);
        config.registerSerialization(PointAccumulator.class);
        config.registerSerialization(HierarchicalArray2D.class);
        config.registerSerialization(Array2D.class);
        config.registerSerialization(Position.class);
        config.registerSerialization(Object[][].class);
        config.registerSerialization(TransferMap.class);
        config.registerSerialization(ParticleMaps.class);
        config.registerSerialization(MapCell.class);
        config.registerSerialization(LaserScan.class);
        config.registerSerialization(ParticleValue.class);
        config.registerSerialization(TNodeValue.class);
        config.registerSerialization(RangeSensor.class);
        config.registerSerialization(RangeReading.class);
    }

    private static class RabbitMQStaticBoltConfigurator implements BoltConfigurator {

        @Override
        public MessageBuilder getMessageBuilder() {
            return new DefaultRabbitMQMessageBuilder();
        }

        @Override
        public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        }

        @Override
        public int queueSize() {
            return 64;
        }

        @Override
        public Map<String, String> getProperties() {
            return new HashMap<String, String>();
        }

        @Override
        public DestinationSelector getDestinationSelector() {
            return new DestinationSelector() {
                @Override
                public String select(Tuple tuple) {
                    return "test";
                }
            };
        }

        @Override
        public DestinationChanger getDestinationChanger() {
            return new StaticDestinations(true);
        }
    }

    private static class RabbitMQStaticSpoutConfigurator implements SpoutConfigurator {
        @Override
        public MessageBuilder getMessageBuilder() {
            return new DefaultRabbitMQMessageBuilder();
        }

        @Override
        public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
            outputFieldsDeclarer.declare(new Fields(Constants.Fields.LASER_SCAN_FIELD, Constants.Fields.SENSOR_ID_FIELD, Constants.Fields.TIME_FIELD));
        }

        @Override
        public int queueSize() {
            return 64;
        }

        @Override
        public Map<String, String> getProperties() {
            return new HashMap<String, String>();
        }

        @Override
        public DestinationChanger getDestinationChanger() {
            return new StaticDestinations(false);
        }
    }

    private static class StaticDestinations implements DestinationChanger {
        private DestinationChangeListener dstListener;

        private boolean sender;

        private StaticDestinations(boolean sender) {
            this.sender = sender;
        }

        @Override
        public void start() {
            StreamTopologyBuilder streamTopologyBuilder = new StreamTopologyBuilder();
            StreamComponents components = streamTopologyBuilder.buildComponents();
            Map conf = components.getConf();
            String url = (String) conf.get(Constants.RABBITMQ_URL);
            DestinationConfiguration configuration = new DestinationConfiguration("rabbitmq", url, "test", "test");
            configuration.setGrouped(true);
            if (!sender) {
                configuration.addProperty("queueName", "laser_scan");
                configuration.addProperty("routingKey", "laser_scan");
                configuration.addProperty("exchange", "simbard_laser");
            } else {
                configuration.addProperty("queueName", "map");
                configuration.addProperty("routingKey", "map");
                configuration.addProperty("exchange", "simbard_map");
            }

            dstListener.addDestination("rabbitmq", configuration);
            dstListener.addPathToDestination("rabbitmq", "test");
        }

        @Override
        public void stop() {
            dstListener.removeDestination("rabbitmq");
        }

        @Override
        public void registerListener(DestinationChangeListener destinationChangeListener) {
            this.dstListener = destinationChangeListener;
        }

        @Override
        public void setTask(int i, int i2) {

        }

        @Override
        public int getTaskIndex() {
            return 0;
        }

        @Override
        public int getTotalTasks() {
            return 0;
        }
    }
}
