package cgl.iotrobots.slam.streaming;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.IRichSpout;
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
        options.addOption(Constants.ARGS_PARALLEL, true, "No of parallel nodes");

        CommandLineParser commandLineParser = new BasicParser();
        CommandLine cmd = commandLineParser.parse(options, args);
        String name = cmd.getOptionValue(Constants.ARGS_NAME);
        boolean local = cmd.hasOption(Constants.ARGS_LOCAL);
        String dsModeValue = cmd.getOptionValue(Constants.ARGS_DS_MODE);
        int dsMode = Integer.parseInt(dsModeValue);
        String pValue = cmd.getOptionValue(Constants.ARGS_PARALLEL);
        int p = Integer.parseInt(pValue);

        StreamTopologyBuilder streamTopologyBuilder;
        if (dsMode == 0) {
            streamTopologyBuilder = new StreamTopologyBuilder();
            buildTestTopology(builder, streamTopologyBuilder, p, false);
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
            conf.setNumWorkers(10);
            StormSubmitter.submitTopology(name, conf, builder.createTopology());
        } else {
            // deploy on a local cluster
            conf.setMaxTaskParallelism(10);
            LocalCluster cluster = new LocalCluster();
            cluster.submitTopology("drone", conf, builder.createTopology());
            Thread.sleep(1000000);
            cluster.shutdown();
        }
    }

    private static void buildTestTopology(TopologyBuilder builder, StreamTopologyBuilder streamTopologyBuilder, int parallel, boolean iotCloud) {
        // first create a rabbitmq Spout
        ErrorReporter reporter = new ErrorReporter() {
            @Override
            public void reportError(Throwable throwable) {
                LOG.error("error occured", throwable);
            }
        };
        IRichSpout dataSpout;
        IRichSpout controlSpout;
        IRichBolt mapSendBolt;
        IRichBolt valueSendBolt;
        if (!iotCloud) {
            dataSpout = new RabbitMQSpout(new RabbitMQStaticSpoutConfigurator(0), reporter);
            controlSpout = new RabbitMQSpout(new RabbitMQStaticSpoutConfigurator(3), reporter);
            mapSendBolt = new RabbitMQBolt(new RabbitMQStaticBoltConfigurator(1), reporter);
            valueSendBolt = new RabbitMQBolt(new RabbitMQStaticBoltConfigurator(2), reporter);
        } else {
            StreamComponents components = streamTopologyBuilder.buildComponents();
            dataSpout = components.getSpouts().get(Constants.Topology.RECEIVE_SPOUT);
            controlSpout = components.getSpouts().get(Constants.Topology.CONTROL_SPOUT);
            mapSendBolt = components.getBolts().get(Constants.Topology.MAP_SEND_BOLD);
            valueSendBolt = components.getBolts().get(Constants.Topology.BEST_PARTICLE_SEND_BOLT);
        }

        ScanMatchBolt scanMatchBolt = new ScanMatchBolt();
        ReSamplingBolt reSamplingBolt = new ReSamplingBolt();
        MapBuildingBolt mapBuildingBolt = new MapBuildingBolt();

        builder.setSpout(Constants.Topology.RECEIVE_SPOUT, dataSpout, 1);
        builder.setSpout(Constants.Topology.CONTROL_SPOUT, controlSpout, 1);
        builder.setBolt(Constants.Topology.SCAN_MATCH_BOLT, scanMatchBolt, parallel).allGrouping(Constants.Topology.RECEIVE_SPOUT).allGrouping(Constants.Topology.CONTROL_SPOUT, Constants.Fields.CONTROL_STREAM);
        builder.setBolt(Constants.Topology.RE_SAMPLE_BOLT, reSamplingBolt, 1).shuffleGrouping(Constants.Topology.SCAN_MATCH_BOLT, Constants.Fields.PARTICLE_STREAM).allGrouping(Constants.Topology.CONTROL_SPOUT, Constants.Fields.CONTROL_STREAM);;
        builder.setBolt(Constants.Topology.MAP_COMPUTE_BOLT, mapBuildingBolt, 1).shuffleGrouping(Constants.Topology.SCAN_MATCH_BOLT, Constants.Fields.MAP_STREAM);
        builder.setBolt(Constants.Topology.MAP_SEND_BOLD, mapSendBolt, 1).shuffleGrouping(Constants.Topology.MAP_COMPUTE_BOLT);
        builder.setBolt(Constants.Topology.BEST_PARTICLE_SEND_BOLT, valueSendBolt, 1).shuffleGrouping(Constants.Topology.SCAN_MATCH_BOLT, Constants.Fields.BEST_PARTICLE_STREAM);
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

        int bolt;

        private RabbitMQStaticBoltConfigurator(int bolt) {
            this.bolt = bolt;
        }

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
            return new StaticDestinations(bolt);
        }
    }

    private static class RabbitMQStaticSpoutConfigurator implements SpoutConfigurator {
        int spout;

        private RabbitMQStaticSpoutConfigurator(int spout) {
            this.spout = spout;
        }

        @Override
        public MessageBuilder getMessageBuilder() {
            return new DefaultRabbitMQMessageBuilder();
        }

        @Override
        public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
            if (spout == 0) {
                outputFieldsDeclarer.declare(new Fields(Constants.Fields.BODY,
                        Constants.Fields.SENSOR_ID_FIELD, Constants.Fields.TIME_FIELD));
            } else {
                outputFieldsDeclarer.declareStream(Constants.Fields.CONTROL_STREAM,
                        new Fields(Constants.Fields.BODY,
                                Constants.Fields.SENSOR_ID_FIELD, Constants.Fields.TIME_FIELD));
            }
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
            return new StaticDestinations(spout);
        }

        @Override
        public String getStream() {
            if (spout == 0) {
                return null;
            } else {
                return Constants.Fields.CONTROL_STREAM;
            }
        }
    }

    private static class StaticDestinations implements DestinationChanger {
        private DestinationChangeListener dstListener;

        private int sender;

        private StaticDestinations(int sender) {
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
            if (sender == 0) {
                configuration.addProperty("queueName", "laser_scan");
                configuration.addProperty("routingKey", "laser_scan");
                configuration.addProperty("exchange", "simbard_laser");
            } else if (sender == 1){
                configuration.addProperty("queueName", "map");
                configuration.addProperty("routingKey", "map");
                configuration.addProperty("exchange", "simbard_map");
            } else if (sender == 2) {
                configuration.addProperty("queueName", "best");
                configuration.addProperty("routingKey", "best");
                configuration.addProperty("exchange", "simbard_best");
            } else if (sender == 3) {
                configuration.addProperty("queueName", "control");
                configuration.addProperty("routingKey", "control");
                configuration.addProperty("exchange", "simbard_control");
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
