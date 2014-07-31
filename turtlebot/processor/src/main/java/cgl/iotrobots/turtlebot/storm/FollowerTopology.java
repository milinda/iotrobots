package cgl.iotrobots.turtlebot.storm;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.turtlebot.commons.CommonsUtils;
import cgl.iotrobots.turtlebot.commons.Motion;
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
        options.addOption(Constants.ARGS_URL, true, "URL of the AMQP Broker");
        options.addOption(Constants.ARGS_NAME, true, "Name of the topology");
        options.addOption(Constants.ARGS_LOCAL, false, "Weather we want run locally");
        options.addOption(Constants.ARGS_DS_MODE, true, "The distributed mode, specify 0, 1, 2, 3 etc");

        CommandLineParser commandLineParser = new BasicParser();
        CommandLine cmd = commandLineParser.parse(options, args);
        String url = cmd.getOptionValue(Constants.ARGS_URL);
        String name = cmd.getOptionValue(Constants.ARGS_NAME);
        boolean local = cmd.hasOption(Constants.ARGS_LOCAL);
        String dsModeValue = cmd.getOptionValue(Constants.ARGS_DS_MODE);
        int dsMode = Integer.parseInt(dsModeValue);

        if (dsMode == 0) {
            buildAllInOneTopology(builder, r, url);
        } else if (dsMode == 1) {
            buildAllSeparateTopology(builder, r, url);
        } else if (dsMode == 2) {
            buildDecodeAndTrackingTopology(builder, r, url);
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

    private static void buildAllInOneTopology(TopologyBuilder builder, ErrorReporter r, String url) {
        builder.setSpout(Constants.KINECT_FRAME_RECV, new RabbitMQSpout(new SpoutConfigurator(url), r), 1);
        builder.setBolt(Constants.OBJECT_DETECTION, new ObjectDetectionBolt(true)).shuffleGrouping(Constants.KINECT_FRAME_RECV);
        builder.setBolt(Constants.SEND_BOLT, new RabbitMQBolt(new BoltConfigurator(url), r)).shuffleGrouping(Constants.OBJECT_DETECTION);
    }

    private static void buildAllSeparateTopology(TopologyBuilder builder, ErrorReporter r, String url) {
        builder.setSpout(Constants.KINECT_FRAME_RECV, new RabbitMQSpout(new SpoutConfigurator(url), r), 1);
        builder.setBolt(Constants.UNCOMPRESS_BOLT, new UncompressBolt()).shuffleGrouping(Constants.KINECT_FRAME_RECV);
        builder.setBolt(Constants.OBJECT_DETECTION, new ObjectDetectionBolt(false)).shuffleGrouping(Constants.UNCOMPRESS_BOLT);
        builder.setBolt(Constants.SEND_BOLT, new RabbitMQBolt(new BoltConfigurator(url), r)).shuffleGrouping(Constants.OBJECT_DETECTION);
    }

    private static void buildDecodeAndTrackingTopology(TopologyBuilder builder, ErrorReporter r, String url) {

    }

    private static class TurtleMessageBuilder implements MessageBuilder {
        /**
         * We get the kinect message here
         * @param message kinect
         * @return bytes
         */
        @Override
        public List<Object> deSerialize(RabbitMQMessage message) {
            System.out.println("Got kinect message");

            Object sensorId = null;
            Object time = null;
            Map<String, Object> props = new HashMap<String, Object>();
            AMQP.BasicProperties properties = message.getProperties();
            if (properties != null && properties.getHeaders() != null) {
                sensorId = properties.getHeaders().get(TransportConstants.SENSOR_ID);
                time = properties.getHeaders().get("time");
            }

            byte []body = message.getBody();
            List<Object> tuples = new ArrayList<Object>();
            tuples.add(body);
            if (sensorId != null) {
                tuples.add(sensorId.toString());
            }

            if (time != null) {
                tuples.add(time.toString());
            }
            return tuples;
        }

        /**
         * We get the control message and create a RabbitMQmessage
         * @param tuple containing control message
         * @return control message
         */
        @Override
        public RabbitMQMessage serialize(Tuple tuple) {
            Motion motion = (Motion) tuple.getValueByField("control");
            String sensorId = (String) tuple.getValueByField("sensorID");
            String time = (String) tuple.getValueByField("time");

            byte []body;
            try {
                Map<String, Object> props = new HashMap<String, Object>();
                props.put(TransportConstants.SENSOR_ID, sensorId);
                props.put("time", time);

                // System.out.println("Sending message" + motion);
                body = CommonsUtils.motionToJSON(motion);
                return new RabbitMQMessage(null, null, null, new AMQP.BasicProperties.Builder().headers(props).build(), body);
            } catch (IOException e) {
                LOG.error("Failed to convert Motion to json", e);
            }
            return null;
        }
    }

    private static class SpoutConfigurator implements RabbitMQConfigurator {
        private String url = "amqp://10.39.1.16:5672";

        private SpoutConfigurator(String url) {
            this.url = url;
        }

        @Override
        public String getURL() {
            return url;
        }

        @Override
        public boolean isAutoAcking() {
            return true;
        }

        @Override
        public int getPrefetchCount() {
            return 1024;
        }

        @Override
        public boolean isReQueueOnFail() {
            return false;
        }

        @Override
        public String getConsumerTag() {
            return "sender";
        }

        @Override
        public List<RabbitMQDestination> getQueueName() {
            List<RabbitMQDestination> list = new ArrayList<RabbitMQDestination>();
            list.add(new RabbitMQDestination("local-1.turtle_storm_frames", "turtle", "turtle_storm_frames"));
            return list;
        }

        @Override
        public MessageBuilder getMessageBuilder() {
            return new TurtleMessageBuilder();
        }

        @Override
        public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
            outputFieldsDeclarer.declare(new Fields("image_frame", "sensorID", "time"));
        }

        @Override
        public int queueSize() {
            return 1024;
        }

        @Override
        public RabbitMQDestinationSelector getDestinationSelector() {
            return null;
        }
    }

    private static class BoltConfigurator implements RabbitMQConfigurator {
        private String url = "amqp://10.39.1.16:5672";

        private BoltConfigurator(String url) {
            this.url = url;
        }

        @Override
        public String getURL() {
            return url;
        }

        @Override
        public boolean isAutoAcking() {
            return true;
        }

        @Override
        public int getPrefetchCount() {
            return 1024;
        }

        @Override
        public boolean isReQueueOnFail() {
            return false;
        }

        @Override
        public String getConsumerTag() {
            return "control";
        }

        @Override
        public List<RabbitMQDestination> getQueueName() {
            List<RabbitMQDestination> list = new ArrayList<RabbitMQDestination>();
            list.add(new RabbitMQDestination("local-1.turtle_storm_control", "turtle", "turtle_storm_control"));
            return list;
        }

        @Override
        public MessageBuilder getMessageBuilder() {
            return new TurtleMessageBuilder();
        }

        @Override
        public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
            outputFieldsDeclarer.declare(new Fields("control"));
        }

        @Override
        public int queueSize() {
            return 1024;
        }

        @Override
        public RabbitMQDestinationSelector getDestinationSelector() {
            return new RabbitMQDestinationSelector() {
                @Override
                public String select(Tuple tuple) {
                    return "local-1.turtle_storm_control";
                }
            };
        }
    }
}
