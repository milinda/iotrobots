package cgl.iotrobots.slam.streaming;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.topology.TopologyBuilder;
import cgl.iotrobots.slam.core.grid.Array2D;
import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.grid.HierarchicalArray2D;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.gridfastsalm.TNode;
import cgl.iotrobots.slam.core.utils.DoublePoint;
import cgl.iotrobots.slam.core.utils.IntPoint;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import org.apache.commons.cli.BasicParser;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.Options;

public class SLAMTopology {
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
            buildAllInOneTopology(builder, streamTopologyBuilder);
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
            conf.setNumWorkers(5);
            StormSubmitter.submitTopology(name, conf, builder.createTopology());
        } else {
            // deploy on a local cluster
            conf.setMaxTaskParallelism(3);
            LocalCluster cluster = new LocalCluster();
            cluster.submitTopology("drone", conf, builder.createTopology());
            Thread.sleep(1000000);
            cluster.shutdown();
        }
    }

    private static void buildAllInOneTopology(TopologyBuilder builder, StreamTopologyBuilder streamTopologyBuilder) {
        StreamComponents components = streamTopologyBuilder.buildComponents();
    }

    private static void addSerializers(Config config) {
        config.registerSerialization(DoublePoint.class);
        config.registerSerialization(IntPoint.class);
        config.registerSerialization(Particle.class);
        config.registerSerialization(GMap.class);
        config.registerSerialization(Array2D.class);
        config.registerSerialization(HierarchicalArray2D.class);
        config.registerSerialization(TNode.class);
    }
}
