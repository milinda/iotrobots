package cgl.iotrobots.st.storm;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.task.ShellBolt;
import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import com.rabbitmq.client.AMQP;
import com.ss.rabbitmq.*;
import com.ss.rabbitmq.bolt.RabbitMQBolt;
import org.apache.commons.cli.BasicParser;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.Options;
import org.apache.commons.codec.binary.Base64;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SphereTrackingTopology {
    private static Logger LOG = LoggerFactory.getLogger(SphereTrackingTopology.class);

    public static class ImageProcessing extends ShellBoltN implements IRichBolt {
        public ImageProcessing() {
            super("python", "DroneFrameProcess.py");
        }

        @Override
        public void declareOutputFields(OutputFieldsDeclarer declarer) {
            declarer.declare(new Fields("image", "time"));
        }

        @Override
        public Map<String, Object> getComponentConfiguration() {
            return null;
        }
    }

    public static void main(String[] args) throws Exception {
        TopologyBuilder builder = new TopologyBuilder();
        ErrorReporter r = new ErrorReporter() {
            @Override
            public void reportError(Throwable t) {
                t.printStackTrace();
            }
        };

        Options options = new Options();
        options.addOption("url", true, "URL of the AMQP Broker");
        options.addOption("name", true, "Name of the topology");
        options.addOption("local", false, "Weather we want run locally");

        CommandLineParser commandLineParser = new BasicParser();
        CommandLine cmd = commandLineParser.parse(options, args);
        String url = cmd.getOptionValue("url");
        String name = cmd.getOptionValue("name");
        boolean local = cmd.hasOption("local");

        builder.setSpout("frame_receive", new RabbitMQSpout(new SpoutConfigurator(url), r), 1);
        builder.setBolt("decode_process", new ImageProcessing()).shuffleGrouping("frame_receive");
        builder.setBolt("send_command", new RabbitMQBolt(new BoltConfigurator(url), r)).shuffleGrouping("decode_process");

        Config conf = new Config();
        conf.setDebug(false);

        // we are going to deploy on a real cluster
        if (!local) {
            conf.setNumWorkers(3);
            StormSubmitter.submitTopology(name, conf, builder.createTopology());
        } else {
            // deploy on a local cluster
            conf.setMaxTaskParallelism(3);
            LocalCluster cluster = new LocalCluster();
            cluster.submitTopology("image_proc", conf, builder.createTopology());
            Thread.sleep(1000000);
            cluster.shutdown();
        }
    }

    private static class Base64Builder implements MessageBuilder {
        @Override
        public List<Object> deSerialize(RabbitMQMessage message) {
            Object time = null;
            AMQP.BasicProperties properties = message.getProperties();
            if (properties != null && properties.getHeaders() != null) {
                time = properties.getHeaders().get("time");
            }

            byte []body = message.getBody();
            List<Object> tuples = new ArrayList<Object>();
            String encodedBytes = Base64.encodeBase64String(body);
            tuples.add(encodedBytes);

            if (time != null) {
                tuples.add(time.toString());
            }

            return tuples;
        }

        @Override
        public RabbitMQMessage serialize(Tuple tuple) {
            String time = (String) tuple.getValueByField("time");
            Map<String, Object> props = new HashMap<String, Object>();
            props.put("time", time);

            // System.out.println("Sending message" + motion);
            return new RabbitMQMessage(null, null, null,
                    new AMQP.BasicProperties.Builder().headers(props).build(),
                    tuple.getValue(0).toString().getBytes());
        }
    }

    private static class SpoutConfigurator implements RabbitMQConfigurator {
        private String url = "amqp://localhost:5672";

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
            list.add(new RabbitMQDestination("local-1.storm_drone_frame", "storm_drone", "storm_drone_frame"));
            return list;
        }

        @Override
        public MessageBuilder getMessageBuilder() {
            return new Base64Builder();
        }

        @Override
        public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
            outputFieldsDeclarer.declare(new Fields("frame", "time"));
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
        private String url = "amqp://localhost:5672";

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
            list.add(new RabbitMQDestination("local-1.storm_control", "storm_drone", "storm_control"));
            return list;
        }

        @Override
        public MessageBuilder getMessageBuilder() {
            return new Base64Builder();
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
                    return "local-1.storm_control";
                }
            };
        }
    }
}
