package cgl.iotrobots.turtlebot.storm;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.IRichSpout;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.turtlebot.commons.CommonsUtils;
import cgl.iotrobots.turtlebot.commons.Motion;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import com.rabbitmq.client.AMQP;
import com.ss.rabbitmq.*;
import com.ss.rabbitmq.bolt.RabbitMQBolt;
import org.apache.commons.cli.BasicParser;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.Options;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FollowerTopology {
    private static Logger LOG = LoggerFactory.getLogger(FollowerTopology.class);

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

        IRichSpout spout = components.getSpouts().get(Constants.KINECT_FRAME_RECV);
        IRichBolt bolt = components.getBolts().get(Constants.SEND_BOLT);

        builder.setSpout(Constants.KINECT_FRAME_RECV, spout, 1);
        builder.setBolt(Constants.OBJECT_DETECTION, new ObjectDetectionBolt(true)).shuffleGrouping(Constants.KINECT_FRAME_RECV);
        builder.setBolt(Constants.SEND_BOLT, bolt).shuffleGrouping(Constants.OBJECT_DETECTION);
    }

    private static void buildAllSeparateTopology(TopologyBuilder builder) {
        StreamTopologyBuilder streamTopologyBuilder = new StreamTopologyBuilder();
        StreamComponents components = streamTopologyBuilder.buildComponents();

        IRichSpout spout = components.getSpouts().get(Constants.KINECT_FRAME_RECV);
        IRichBolt bolt = components.getBolts().get(Constants.SEND_BOLT);

        builder.setSpout(Constants.KINECT_FRAME_RECV, spout, 1);
        builder.setBolt(Constants.UNCOMPRESS_BOLT, new UncompressBolt()).shuffleGrouping(Constants.KINECT_FRAME_RECV);
        builder.setBolt(Constants.OBJECT_DETECTION, new ObjectDetectionBolt(false)).shuffleGrouping(Constants.UNCOMPRESS_BOLT);
        builder.setBolt(Constants.SEND_BOLT, bolt).shuffleGrouping(Constants.OBJECT_DETECTION);
    }

    private static void buildDecodeAndTrackingTopology(TopologyBuilder builder) {

    }
}
