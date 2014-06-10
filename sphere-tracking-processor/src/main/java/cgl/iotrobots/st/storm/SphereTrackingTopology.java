package cgl.iotrobots.st.storm;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.task.ShellBolt;
import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import com.ss.rabbitmq.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class SphereTrackingTopology {
    public static class SplitSentence extends ShellBolt implements IRichBolt {
        public SplitSentence() {
            super("python", "image_proc.py");
        }

        @Override
        public void declareOutputFields(OutputFieldsDeclarer declarer) {
            declarer.declare(new Fields("image"));
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

        builder.setSpout("spout", new RabbitMQSpout(new SpoutConfigurator(), r), 3);
        builder.setBolt("print", new SplitSentence()).shuffleGrouping("spout");

        Config conf = new Config();
        conf.setDebug(true);


        if (args != null && args.length > 0) {
            conf.setNumWorkers(3);
            StormSubmitter.submitTopology(args[0], conf, builder.createTopology());
        } else {
            conf.setMaxTaskParallelism(3);
            LocalCluster cluster = new LocalCluster();
            cluster.submitTopology("word-count", conf, builder.createTopology());
            Thread.sleep(10000);
            cluster.shutdown();
        }
    }

    private static class TimeStampMessageBuilder implements MessageBuilder {
        @Override
        public List<Object> deSerialize(RabbitMQMessage message) {
            byte []body = message.getBody();
            List<Object> tuples = new ArrayList<Object>();
            tuples.add(body);
            return tuples;
        }

        @Override
        public RabbitMQMessage serialize(Tuple tuple) {
            return null;
        }
    }

    private static class SpoutConfigurator implements RabbitMQConfigurator {
        private String url = "amqp://localhost:5672";

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
            list.add(new RabbitMQDestination("sender", "sphere_tracking", "sender"));
            return list;
        }

        @Override
        public MessageBuilder getMessageBuilder() {
            return new TimeStampMessageBuilder();
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
}
