package test;

import cgl.iotrobots.collavoid.LocalPlanner.LocalPlanner;
import cgl.iotrobots.collavoid.ROSAgent.Agent;
import costmap_2d.VoxelGrid;
import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.rosjava.tf.pubsub.TransformListener;

import java.net.InetAddress;
import java.net.URI;
import java.net.UnknownHostException;

import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.regex.PatternSyntaxException;


public class runAgentAlone {

    public static TransformListener tf;
    private static Logger logger = Logger.getLogger("runAgentAlone");
    public static VoxelGrid cosmap_voxelGrid;

    public static void main(String[] args)throws InterruptedException{

        String IP = null;
        String hostname = null;

        try {
            InetAddress ia = InetAddress.getLocalHost();
            hostname = ia.getHostName();
            IP = ia.getHostAddress();
        } catch (UnknownHostException e) {
            e.printStackTrace();
        }

        hostname=StringFilter(hostname);//replace all non-alphabet and non-number character into _

        NodeConfiguration configuration = NodeConfiguration.newPublic(IP.toString(),
                URI.create("http://localhost:11311"));

        final NodeMainExecutor runner= DefaultNodeMainExecutor.newDefault();
        //test agent
//        String agentId=new String("agent_"+hostname);
//        logger.info("My name is: " + agentId);
//        Agent agent=new Agent(agentId,tf);
//        configuration.setNodeName(agentId);
//        runner.execute(agent,configuration);

        LocalPlanner localPlanner=new LocalPlanner(hostname,tf,cosmap_voxelGrid);
        String localPlannerId=new String("localPlanner_"+hostname);
        configuration.setNodeName(localPlannerId);
        runner.execute(localPlanner,configuration);

        //setParameters setParams=new setParameters();
        //runner.execute(setParams, configuration);

    }

    public   static   String StringFilter(String   str)   throws PatternSyntaxException {
        //clear special characters
        String regEx="[`~!@#$%^&*()+=|{}':;',\\[\\].<>/?~！@#￥%……&*（）——+|{}【】‘；：”“’。，、？-]";
        Pattern p   =   Pattern.compile(regEx);
        Matcher m   =   p.matcher(str);
        return   m.replaceAll("_").trim();
    }
}
