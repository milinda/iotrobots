package cgl.iotrobots.collavoid.iotTopology;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import cgl.iotrobots.collavoid.commons.storm.Methods_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class runIotTopology {

    private static Logger LOG = LoggerFactory.getLogger(runIotTopology.class);

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
            final BuildIotTopology topology = new BuildIotTopology(conf);
            StormSubmitter.submitTopology(args[0], conf, topology.getStormTopology());
            LOG.info("Planner started. Running on the cluster!!");
        } else {
            // deploy on a local cluster
            conf.setMaxTaskParallelism(3);
            final LocalCluster cluster = new LocalCluster("localhost", new Long(2181));
            final BuildIotTopology topology = new BuildIotTopology(cluster, conf, "default");
            topology.setTopology();
            topology.submit();

            Thread.sleep(1000000);
            cluster.shutdown();
        }
    }


}
