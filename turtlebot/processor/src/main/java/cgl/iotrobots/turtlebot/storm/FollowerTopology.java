package cgl.iotrobots.turtlebot.storm;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.turtlebot.commons.CommonsUtils;
import cgl.iotrobots.turtlebot.commons.Motion;
import com.ss.rabbitmq.*;
import com.ss.rabbitmq.bolt.RabbitMQBolt;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

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

        builder.setSpout("recv_spout", new RabbitMQSpout(new SpoutConfigurator(), r), 1);
        builder.setBolt("image_proc", new ObjectDetectionBolt()).shuffleGrouping("recv_spout");
        builder.setBolt("send_bolt", new RabbitMQBolt(new BoltConfigurator(), r)).shuffleGrouping("image_proc");

        Config conf = new Config();
        conf.setDebug(false);

        if (args != null && args.length > 0) {
            conf.setNumWorkers(3);
            StormSubmitter.submitTopology(args[0], conf, builder.createTopology());
        } else {
            conf.setMaxTaskParallelism(3);
            LocalCluster cluster = new LocalCluster();
            cluster.submitTopology("image_proc", conf, builder.createTopology());
            Thread.sleep(1000000);
            cluster.shutdown();
        }
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
            byte []body = message.getBody();
            List<Object> tuples = new ArrayList<Object>();
            tuples.add(body);
            return tuples;
        }

        /**
         * We get the control message and create a RabbitMQmessage
         * @param tuple containing control message
         * @return control message
         */
        @Override
        public RabbitMQMessage serialize(Tuple tuple) {
            Motion motion = (Motion) tuple.getValue(0);
            byte []body;
            try {
                // System.out.println("Sending message" + motion);
                body = CommonsUtils.motionToJSON(motion);
                return new RabbitMQMessage(null, null, null, null, body);
            } catch (IOException e) {
                LOG.error("Failed to convert Motion to json", e);
            }
            return null;
        }
    }

    private static class SpoutConfigurator implements RabbitMQConfigurator {
        private String url = "amqp://10.39.1.16:5672";

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
            list.add(new RabbitMQDestination("local-1.kinect", "turtle_sensor", "kinect"));
            return list;
        }

        @Override
        public MessageBuilder getMessageBuilder() {
            return new TurtleMessageBuilder();
        }

        @Override
        public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
            outputFieldsDeclarer.declare(new Fields("time1"));
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
            list.add(new RabbitMQDestination("local-1.control", "turtle_sensor", "turtle"));
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
                    return "local-1.control";
                }
            };
        }
    }
}
