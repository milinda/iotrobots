package cgl.iotrobots.collavoid.topology;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import cgl.iotrobots.collavoid.commons.storm.Methods_storm;
import org.apache.commons.cli.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.Map;


public class run {

    private static Logger LOG = LoggerFactory.getLogger(run.class);
    private static String COMPUTATION_BOLT_PARALLELISM_ARG = "pc";
    private static String AGENT_STATE_PARALLELISM_ARG = "ps";
    private static String TOPOLOGY_NAME = "n";
    private static String REMOTE_MODE = "r";
    private static String topologyName = "default";

    public static void main(String[] args) throws Exception {
        Config conf = new Config();
        conf.setDebug(false);
        conf.put(Config.TOPOLOGY_ACKER_EXECUTORS, 0);// very important

        // add the serializers
        Methods_storm.addSerializers(conf);

        Map<String, String> props = getProperties(args);

        Integer paraCompute = null;
        Integer paraState = null;

        if (props != null) {
            if (props.get(COMPUTATION_BOLT_PARALLELISM_ARG) != null) {
                paraCompute = Integer.parseInt(props.get(COMPUTATION_BOLT_PARALLELISM_ARG));
            }
            if (props.get(AGENT_STATE_PARALLELISM_ARG) != null) {
                paraState = Integer.parseInt(props.get(AGENT_STATE_PARALLELISM_ARG));
            }
            if (props.get(TOPOLOGY_NAME) != null) {
                topologyName = props.get(TOPOLOGY_NAME);
            }
        }

        if (props == null ||! props.containsKey(REMOTE_MODE)) {
            // deploy on a local cluster
            conf.setMaxTaskParallelism(3);
            final LocalCluster cluster = new LocalCluster();
            final CATopology topology = new CATopology(cluster, conf, topologyName);
            topology.buildCATopology(paraCompute, paraState);
            topology.submit();
            LOG.info("Topology {} start running!", topologyName);
            Thread.sleep(1000000);
            LOG.info("Stopping topology...........................!!");
            topology.shutdown();
        } else {
            // deploy on a real cluster
            conf.setNumWorkers(20);
            final CATopology topology = new CATopology(conf);
            StormSubmitter.submitTopology(topologyName, conf, topology.buildCATopology(paraCompute, paraState));
            LOG.info("\n********************Planner started. Running on the cluster!!***********************");
        }
    }

    private static Map<String, String> getProperties(String[] args) {
        if (args == null || args.length == 0) {
            return null;
        }
        Map<String, String> conf = new HashMap<String, String>();

        Options options = new Options();
        options.addOption(COMPUTATION_BOLT_PARALLELISM_ARG, true, "Set computation bolt parallelism!");
        options.addOption(AGENT_STATE_PARALLELISM_ARG, true, "Set agent state bolt parallelism!");
        options.addOption(TOPOLOGY_NAME, true, "Set topology name!");
        options.addOption(REMOTE_MODE, false, "Run in remote mode!");

        CommandLineParser commandLineParser = new BasicParser();
        try {
            CommandLine cmd = commandLineParser.parse(options, args);
            String p = cmd.getOptionValue(COMPUTATION_BOLT_PARALLELISM_ARG);
            conf.put(COMPUTATION_BOLT_PARALLELISM_ARG, p);
            p = cmd.getOptionValue(AGENT_STATE_PARALLELISM_ARG);
            conf.put(AGENT_STATE_PARALLELISM_ARG, p);
            p = cmd.getOptionValue(TOPOLOGY_NAME);
            conf.put(TOPOLOGY_NAME, p);
            if (cmd.hasOption(REMOTE_MODE)) {
                conf.put(REMOTE_MODE, REMOTE_MODE);
            }
            return conf;
        } catch (ParseException e) {
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp("topology", options);
        }
        return null;
    }

}
