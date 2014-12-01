package test.AgentTest;


import cgl.iotrobots.collavoid.ROSAgent.ROSAgent;
import costmap_2d.VoxelGrid;
import org.ros.node.*;
import org.ros.rosjava.tf.pubsub.TransformListener;
import test.node.PositionShareMux;

import java.net.InetAddress;
import java.net.URI;
import java.net.UnknownHostException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.regex.PatternSyntaxException;


/**
 * Created by hjh on 11/16/14.
 */
public class PositionShareCallbackTest {

        public static TransformListener tf;
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

        String agentId="robot1";
        //String agentId=new String("agent_"+hostname);

        //execute
        NodeConfiguration configuration = NodeConfiguration.newPublic(IP.toString(),
                URI.create("http://localhost:11311"));
        final NodeMainExecutor runner= DefaultNodeMainExecutor.newDefault();
        tf=null;

        //start position mux to simulate position publish
        PositionShareMux psmux=new PositionShareMux();
        runner.execute(psmux,configuration);

        Thread.sleep(5000);

        //start ROSAgent
//        configuration.setNodeName(agentId);
//        ROSAgent ROSAgent =new ROSAgent(agentId,tf);
//        runner.execute(ROSAgent,configuration);

    }

    public   static   String StringFilter(String   str)   throws PatternSyntaxException {
        //clear special characters
        String regEx="[`~!@#$%^&*()+=|{}':;',\\[\\].<>/?~！@#￥%……&*（）——+|{}【】‘；：”“’。，、？-]";
        Pattern p   =   Pattern.compile(regEx);
        Matcher m   =   p.matcher(str);
        return   m.replaceAll("_").trim();
    }
}




