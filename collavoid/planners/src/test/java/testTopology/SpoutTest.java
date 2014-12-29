package testTopology;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.rmqmsg.Header_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Pose_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import cgl.sensorstream.core.rabbitmq.DefaultRabbitMQMessageBuilder;
import com.ss.commons.*;
import com.ss.rabbitmq.ErrorReporter;
import com.ss.rabbitmq.RabbitMQSpout;
import com.ss.rabbitmq.bolt.RabbitMQBolt;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.Map;

public class SpoutTest {

    private static Logger LOG = LoggerFactory.getLogger(SpoutTest.class);

    public static void main(String[] args) throws Exception {
        TopologyBuilder builder = new TopologyBuilder();
        int dsMode = 0;
        boolean local = true;

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
            conf.setNumWorkers(5);
            StormSubmitter.submitTopology("testRMQSpout", conf, builder.createTopology());
        } else {
            // deploy on a local cluster
            conf.setMaxTaskParallelism(3);
            LocalCluster cluster = new LocalCluster();
            cluster.submitTopology("testRMQSpout", conf, builder.createTopology());
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

        ShowOdomBolt showOdomBolt = new ShowOdomBolt();

        builder.setSpout(Constant.TOPOLOGY.ODOMETRY_SPOUT, spout, 1);
        builder.setBolt(Constant.TOPOLOGY.SHOW_ODOMETRY_BOLT, showOdomBolt, 1)
                .shuffleGrouping(Constant.TOPOLOGY.ODOMETRY_SPOUT);

    }

    private static void addSerializers(Config config) {
        config.registerSerialization(Odometry_.class);
        config.registerSerialization(Header_.class);
        config.registerSerialization(Pose_.class);
        config.registerSerialization(Twist_.class);
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
            return new StaticDestinations(false);
        }
    }

    private static class RabbitMQStaticSpoutConfigurator implements SpoutConfigurator {
        @Override
        public MessageBuilder getMessageBuilder() {
            return new DefaultRabbitMQMessageBuilder();
        }

        @Override
        public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
            outputFieldsDeclarer.declare(new Fields(Constant.FIELDS.ODOMETRY_FIELD));
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
            String url = (String) conf.get("rabbitmq_url");
            DestinationConfiguration configuration = new DestinationConfiguration("rabbitmq", url, "test", "robot0");
            configuration.setGrouped(true);
            if (!sender) {
                configuration.addProperty("queueName", "Queue_Odometry");
                configuration.addProperty("routingKey", "RoutingKey_Odometry");
                configuration.addProperty("exchange", "robot0rmq");
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
