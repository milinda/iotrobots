package cgl.iotrobots.collavoid.topologyStreaming;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import cgl.iotrobots.collavoid.commons.planners.Position;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class runTopology {

    private static Logger LOG = LoggerFactory.getLogger(runTopology.class);

    public static void main(String[] args) throws Exception {


        Config conf = new Config();
        conf.setDebug(false);
        // add the serializers
        addSerializers(conf);

        // we are going to deploy on a real cluster
        if (args != null && args.length > 0) {
            conf.setNumWorkers(3);
//            final BuildTopology topology = new BuildTopology(conf);
            final BuildIotTopology topology = new BuildIotTopology(conf);
            StormSubmitter.submitTopology(args[0], conf, topology.getStormTopology());
            LOG.info("Planner started. Running on the cluster!!");
        } else {
            // deploy on a local cluster
            conf.setMaxTaskParallelism(3);
            final LocalCluster cluster = new LocalCluster();
//            final BuildTopology topology = new BuildTopology(cluster, conf, "Collavoid");
            final BuildIotTopology topology = new BuildIotTopology(cluster,conf,"Collavoid");
            topology.setTopology();
            topology.submit();

            Thread.sleep(1000000);
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
        config.registerSerialization(BaseConfig_.class);
        config.registerSerialization(Position.class);
    }

}
