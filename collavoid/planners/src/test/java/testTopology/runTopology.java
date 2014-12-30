package testTopology;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.topology.IRichSpout;
import backtype.storm.topology.TopologyBuilder;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;

import com.rabbitmq.client.ConnectionFactory;
import io.latent.storm.rabbitmq.Declarator;
import io.latent.storm.rabbitmq.RabbitMQSpout;
import io.latent.storm.rabbitmq.config.ConnectionConfig;
import io.latent.storm.rabbitmq.config.ConsumerConfig;
import io.latent.storm.rabbitmq.config.ConsumerConfigBuilder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class runTopology {

    private static Logger LOG = LoggerFactory.getLogger(runTopology.class);

    public static void main(String[] args) throws Exception {
        TopologyBuilder builder = new TopologyBuilder();
        int dsMode = 0;
        boolean local = true;

        if (dsMode == 0) {
            buildTestTopology(builder);
        }

        Config conf = new Config();
        conf.setDebug(false);
        // we are not going to track individual messages, message loss is inherent in the decoder
        // also we cannot replay message because of the decoder
        //conf.put(Config.TOPOLOGY_ACKER_EXECUTORS, 0);

        // add the serializers
        //addSerializers(conf);

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

    private static void buildTestTopology(TopologyBuilder builder) {
        String exchangeName = "robot0_rmq";
//        // first create a rabbitmq Spout
//        ConnectionConfig connectionConfig = new ConnectionConfig("localhost", 5672, "guest", "guest", ConnectionFactory.DEFAULT_VHOST, 10); // host, port, username, password, virtualHost, heartBeat
//        ConsumerConfig spoutConfig = new ConsumerConfigBuilder().connection(connectionConfig)
//                .queue("")
//                .prefetch(200)
//                .requeueOnFail()
//                .build();
//
//        OdometryScheme odometryScheme=new OdometryScheme();
//        Declarator declarator = new CustomStormDeclarator(
//                exchangeName,
//                "Queue_"+Constant_msg.KEY_ODOMETRY,
//                "RoutingKey_" + Constant_msg.KEY_ODOMETRY);
//
//        IRichSpout spout = new RabbitMQSpout(odometryScheme, declarator);
//
//        AttachTimeID showOdomBolt = new AttachTimeID();
//
//        builder.setSpout(Constant.TOPOLOGY.ODOMETRY_SPOUT, spout, 1).
//                addConfigurations(spoutConfig.asMap()).
//                setMaxSpoutPending(200);
//
//        builder.setBolt(Constant.TOPOLOGY.SHOW_ODOMETRY_BOLT, showOdomBolt, 1)
//                .shuffleGrouping(Constant.TOPOLOGY.ODOMETRY_SPOUT);

    }


    private static void addSerializers(Config config) {
        config.registerSerialization(Odometry_.class);
        config.registerSerialization(Header_.class);
        config.registerSerialization(Pose_.class);
        config.registerSerialization(Twist_.class);
        config.registerSerialization(PoseShareMsg_.class);
        config.registerSerialization(PoseArray_.class);
        config.registerSerialization(Vector3d_.class);
        config.registerSerialization(Vector4d_.class);
    }

}
