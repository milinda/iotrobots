package cgl.iotrobots.collavoid.topologyStreaming;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import cgl.iotrobots.collavoid.commons.planners.Position;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.commons.storm.Methods_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class runTopology {

    private static Logger LOG = LoggerFactory.getLogger(runTopology.class);

    public static void main(String[] args) throws Exception {


        Config conf = new Config();
        conf.setDebug(false);
        // add the serializers
        Methods_storm.addSerializers(conf);

        // we are going to deploy on a real cluster
        if (args != null && args.length > 0) {
            conf.setNumWorkers(3);
            final BuildTopology topology = new BuildTopology(conf);
            StormSubmitter.submitTopology(args[0], conf, topology.getStormTopology());
            LOG.info("Planner started. Running on the cluster!!");
        } else {
            // deploy on a local cluster
            conf.setMaxTaskParallelism(3);
            final LocalCluster cluster = new LocalCluster();
            final BuildTopology topology = new BuildTopology(cluster, conf, "Collavoid");
            topology.setTopology();
            topology.submit();

            Thread.sleep(1000000);
            cluster.shutdown();
        }
    }

}
