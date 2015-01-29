package test;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import cgl.iotrobots.collavoid.commons.storm.Methods_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;


public class runIotTopologyTest {

    private static Logger LOG = LoggerFactory.getLogger(runIotTopologyTest.class);

    public static void main(String[] args) throws Exception {

        Config conf = new Config();
        conf.setDebug(false);
        conf.put(Config.STORM_ZOOKEEPER_ROOT, "storm");
        conf.put(Config.TOPOLOGY_ACKER_EXECUTORS,0);
        // add the serializers
        Methods_storm.addSerializers(conf);

        // we are going to deploy on a real cluster
        if (args != null && args.length > 0) {
            conf.setNumWorkers(3);
//            final BuildTopology topology = new BuildTopology(conf);
            final BuildIotTopologyTest topology = new BuildIotTopologyTest(conf);
            StormSubmitter.submitTopology(args[0], conf, topology.getStormTopology());
            LOG.info("Planner started. Running on the cluster!!");
        } else {
            // deploy on a local cluster
            conf.setMaxTaskParallelism(3);
            final LocalCluster cluster = new LocalCluster("localhost", new Long(2181));
//            final BuildTopology topology = new BuildTopology(cluster, conf, "Collavoid");
            final BuildIotTopologyTest topology = new BuildIotTopologyTest(cluster, conf, "default");
            topology.setTopology();
            topology.submit();

            Thread.sleep(1000000);
            cluster.shutdown();
        }
    }


}
