package cgl.iotrobots.perf.proc;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.IRichSpout;
import backtype.storm.topology.TopologyBuilder;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import org.apache.commons.cli.BasicParser;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.Options;

public class PerformanceTopology {
    public static void main(String[] args) throws Exception {
        TopologyBuilder builder = new TopologyBuilder();

        Options options = new Options();
        options.addOption(Constants.ARGS_NAME, true, "Name of the topology");
        options.addOption(Constants.ARGS_LOCAL, false, "Weather we want run locally");
        options.addOption(Constants.ARGS_DS_MODE, true, "The distributed mode, specify 0, 1, 2, 3 etc");
        options.addOption(Constants.ARGS_TRP, true, "The transport to be used, k for Kafka, r for RabbitMQ");

        CommandLineParser commandLineParser = new BasicParser();
        CommandLine cmd = commandLineParser.parse(options, args);
        String name = cmd.getOptionValue(Constants.ARGS_NAME);
        boolean local = cmd.hasOption(Constants.ARGS_LOCAL);
        String dsModeValue = cmd.getOptionValue(Constants.ARGS_DS_MODE);
        int dsMode = Integer.parseInt(dsModeValue);
        String trpValue = cmd.getOptionValue(Constants.ARGS_TRP);

        StreamTopologyBuilder streamTopologyBuilder;

        if (trpValue.equals("k")) {
            streamTopologyBuilder = new StreamTopologyBuilder("kafka_topology.yaml");
        } else {
            streamTopologyBuilder = new StreamTopologyBuilder("rabbitmq_topology.yaml");
        }

        if (dsMode == 0) {
            passThroughTopology(builder, streamTopologyBuilder);
        } else if (dsMode == 1) {
            allInOneTopology(builder, streamTopologyBuilder);
        } else if (dsMode == 2) {
            allSeparateTopology(builder, streamTopologyBuilder);
        }

        Config conf = new Config();
        conf.setDebug(false);
        // we are not going to track individual messages, message loss is inherent in the decoder
        // also we cannot replay message because of the decoder
        conf.put(Config.TOPOLOGY_ACKER_EXECUTORS, 0);

        // we are going to deploy on a real cluster
        if (!local) {
            conf.setNumWorkers(5);
            StormSubmitter.submitTopology(name, conf, builder.createTopology());
        } else {
            // deploy on a local cluster
            conf.setMaxTaskParallelism(3);
            LocalCluster cluster = new LocalCluster();
            cluster.submitTopology("perf", conf, builder.createTopology());
            Thread.sleep(1000000);
            cluster.shutdown();
        }
    }

    private static void passThroughTopology(TopologyBuilder builder, StreamTopologyBuilder streamTopologyBuilder) {
        StreamComponents components = streamTopologyBuilder.buildComponents();

        IRichSpout spout = components.getSpouts().get(Constants.DATA_RECEIVE_SPOUT);
        IRichBolt bolt = components.getBolts().get(Constants.SEND_DATA_BOLT);

        builder.setSpout(Constants.DATA_RECEIVE_SPOUT, spout, 1);
        builder.setBolt(Constants.SEND_DATA_BOLT, bolt).shuffleGrouping(Constants.DATA_RECEIVE_SPOUT);
    }

    private static void allInOneTopology(TopologyBuilder builder, StreamTopologyBuilder streamTopologyBuilder) {
        StreamComponents components = streamTopologyBuilder.buildComponents();

        IRichSpout spout = components.getSpouts().get(Constants.DATA_RECEIVE_SPOUT);
        IRichBolt bolt = components.getBolts().get(Constants.SEND_DATA_BOLT);

        builder.setSpout(Constants.DATA_RECEIVE_SPOUT, spout, 1);
        builder.setBolt(Constants.COMPRESS_DECOMPRESS_BOLT, new CompressDecompressBolt(), 1).shuffleGrouping(Constants.DATA_RECEIVE_SPOUT);
        builder.setBolt(Constants.SEND_DATA_BOLT, bolt).shuffleGrouping(Constants.COMPRESS_DECOMPRESS_BOLT);
    }

    private static void allSeparateTopology(TopologyBuilder builder, StreamTopologyBuilder streamTopologyBuilder) {
        StreamComponents components = streamTopologyBuilder.buildComponents();

        IRichSpout spout = components.getSpouts().get(Constants.DATA_RECEIVE_SPOUT);
        IRichBolt bolt = components.getBolts().get(Constants.SEND_DATA_BOLT);

        builder.setSpout(Constants.DATA_RECEIVE_SPOUT, spout, 1);
        builder.setBolt(Constants.COMPRESS_BOLT, new CompressDecompressBolt(), 1).shuffleGrouping(Constants.DATA_RECEIVE_SPOUT);
        builder.setBolt(Constants.DECOMPRESS_BOLT, new DeCompressionBolt(), 1).shuffleGrouping(Constants.COMPRESS_BOLT);
        builder.setBolt(Constants.SEND_DATA_BOLT, bolt).shuffleGrouping(Constants.DECOMPRESS_BOLT);
    }
}
