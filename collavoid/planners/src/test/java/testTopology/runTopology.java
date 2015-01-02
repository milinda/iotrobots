package testTopology;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.topology.IRichSpout;
import backtype.storm.topology.TopologyBuilder;
import cgl.iotrobots.collavoid.commons.planners.Parameters;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;


public class runTopology {

    private static Logger LOG = LoggerFactory.getLogger(runTopology.class);

    public static void main(String[] args) throws Exception {
        TopologyBuilder builder = new TopologyBuilder();
        int dsMode = 0;
        boolean local = true;

//        if (dsMode == 0) {
//            buildTestTopology(builder);
//        }

        Config conf = new Config();
        conf.setDebug(false);
        // we are not going to track individual messages, message loss is inherent in the decoder
        // also we cannot replay message because of the decoder
        //conf.put(Config.TOPOLOGY_ACKER_EXECUTORS, 0);

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
            List<BuildTopology> topologies = new ArrayList<BuildTopology>();
            for (int i = 0; i < Parameters.ROBOT_NUMBER; i++) {
                topologies.add(new BuildTopology(cluster, conf, i));
                topologies.get(i).setTopology();
                topologies.get(i).submit();
            }
            Thread.sleep(1000000);
            for (BuildTopology topo : topologies) {
                topo.shutdown();
            }
            cluster.shutdown();
        }
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
        config.registerSerialization(Vector2.class);
        config.registerSerialization(StartGoal_.class);
    }

}
