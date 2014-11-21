package test.AgentTest;


import cgl.iotrobots.collavoid.ROSAgent.Agent;
import cgl.iotrobots.collavoid.utils.*;
import collavoid_msgs.pose_twist_covariance_msgs;
import costmap_2d.VoxelGrid;
import geometry_msgs.*;
import nav_msgs.Odometry;
import org.ros.concurrent.CancellableLoop;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.*;
import org.ros.namespace.GraphName;
import org.ros.node.*;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.pubsub.TransformListener;
import sensor_msgs.LaserScan;
import sensor_msgs.PointCloud;
import test.node.PositionShareMux;

import java.net.InetAddress;
import java.net.URI;
import java.net.UnknownHostException;
import java.util.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.regex.PatternSyntaxException;


/**
 * Created by hjh on 11/16/14.
 */
public class agentPositionShareCallback {

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

        //start agent
        configuration.setNodeName(agentId);
        Agent agent=new Agent(agentId,tf);
        runner.execute(agent,configuration);

    }

    public   static   String StringFilter(String   str)   throws PatternSyntaxException {
        //clear special characters
        String regEx="[`~!@#$%^&*()+=|{}':;',\\[\\].<>/?~！@#￥%……&*（）——+|{}【】‘；：”“’。，、？-]";
        Pattern p   =   Pattern.compile(regEx);
        Matcher m   =   p.matcher(str);
        return   m.replaceAll("_").trim();
    }
}




