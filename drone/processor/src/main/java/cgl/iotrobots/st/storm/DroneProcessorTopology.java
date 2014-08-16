package cgl.iotrobots.st.storm;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.IRichSpout;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import com.rabbitmq.client.AMQP;
import com.ss.rabbitmq.*;
import com.ss.rabbitmq.bolt.RabbitMQBolt;
import org.apache.commons.cli.BasicParser;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.Options;
import org.apache.commons.codec.binary.Base64;
import org.jboss.netty.handler.codec.base64.Base64Encoder;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class DroneProcessorTopology {
    public static void main(String[] args) throws Exception {
        TopologyBuilder builder = new TopologyBuilder();
        ErrorReporter r = new ErrorReporter() {
            @Override
            public void reportError(Throwable t) {
                t.printStackTrace();
            }
        };

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

        if (dsMode == 0) {
            buildAllInOneTopology(builder);
        } else if (dsMode == 1) {
            buildAllSeparateTopology(builder);
        } else if (dsMode == 2) {
            buildDecodeAndTrackingTopology(builder);
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
            cluster.submitTopology("drone", conf, builder.createTopology());
            Thread.sleep(1000000);
            cluster.shutdown();
        }
    }

    private static void buildAllInOneTopology(TopologyBuilder builder) {
        StreamTopologyBuilder streamTopologyBuilder = new StreamTopologyBuilder();
        StreamComponents components = streamTopologyBuilder.buildComponents();

        IRichSpout spout = components.getSpouts().get(Constants.FRAME_RECEIVE_SPOUT);
        IRichBolt bolt = components.getBolts().get(Constants.SEND_COMMAND_BOLT);
        IRichSpout navSpout = components.getSpouts().get(Constants.NAV_RECEIVE_SPOUT);

        builder.setSpout(Constants.FRAME_RECEIVE_SPOUT, spout, 1);
        builder.setSpout(Constants.NAV_RECEIVE_SPOUT, navSpout, 1);
        builder.setBolt(Constants.ALL_IN_ONE_BOLT, new DecodeTrackingBolt()).shuffleGrouping(Constants.FRAME_RECEIVE_SPOUT).shuffleGrouping(Constants.NAV_RECEIVE_SPOUT);
        builder.setBolt(Constants.SEND_COMMAND_BOLT, bolt).shuffleGrouping(Constants.ALL_IN_ONE_BOLT);
    }

    private static void buildAllSeparateTopology(TopologyBuilder builder) {
        StreamTopologyBuilder streamTopologyBuilder = new StreamTopologyBuilder();
        StreamComponents components = streamTopologyBuilder.buildComponents();

        IRichSpout spout = components.getSpouts().get(Constants.FRAME_RECEIVE_SPOUT);
        IRichBolt bolt = components.getBolts().get(Constants.SEND_COMMAND_BOLT);
        IRichSpout navSpout = components.getSpouts().get(Constants.NAV_RECEIVE_SPOUT);

        builder.setSpout(Constants.FRAME_RECEIVE_SPOUT, spout, 1);
        builder.setSpout(Constants.NAV_RECEIVE_SPOUT, navSpout, 1);
        builder.setBolt(Constants.DECODE_BOLT, new DecodingBolt()).shuffleGrouping(Constants.FRAME_RECEIVE_SPOUT);
        builder.setBolt(Constants.TRACKING_BOLT, new TrackingBolt()).shuffleGrouping(Constants.DECODE_BOLT);
        builder.setBolt(Constants.PLANING_BOLT, new PlanningBolt()).shuffleGrouping(Constants.TRACKING_BOLT).shuffleGrouping(Constants.NAV_RECEIVE_SPOUT);
        builder.setBolt(Constants.SEND_COMMAND_BOLT, bolt).shuffleGrouping(Constants.PLANING_BOLT);
    }

    private static void buildDecodeAndTrackingTopology(TopologyBuilder builder) {
        StreamTopologyBuilder streamTopologyBuilder = new StreamTopologyBuilder();
        StreamComponents components = streamTopologyBuilder.buildComponents();

        IRichSpout spout = components.getSpouts().get(Constants.FRAME_RECEIVE_SPOUT);
        IRichSpout navSpout = components.getSpouts().get(Constants.NAV_RECEIVE_SPOUT);
        IRichBolt bolt = components.getBolts().get(Constants.SEND_COMMAND_BOLT);

        builder.setSpout(Constants.FRAME_RECEIVE_SPOUT, spout, 1);
        builder.setSpout(Constants.NAV_RECEIVE_SPOUT, navSpout, 1);
        builder.setBolt(Constants.DECODE_AND_TRACKING_BOLT, new DecodeTrackingBolt()).shuffleGrouping(Constants.FRAME_RECEIVE_SPOUT);
        builder.setBolt(Constants.PLANING_BOLT, new PlanningBolt()).shuffleGrouping(Constants.DECODE_AND_TRACKING_BOLT).shuffleGrouping(Constants.NAV_RECEIVE_SPOUT);
        builder.setBolt(Constants.SEND_COMMAND_BOLT, bolt).shuffleGrouping(Constants.PLANING_BOLT);
    }
}
