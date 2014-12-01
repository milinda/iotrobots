package test;

import cgl.iotrobots.collavoid.GlobalPlanner.GlobalPlanner;
import cgl.iotrobots.collavoid.LocalPlanner.LocalPlanner;
import cgl.iotrobots.collavoid.msgmanager.MsgFactory;
import costmap_2d.VoxelGrid;
import geometry_msgs.PoseStamped;
import geometry_msgs.Twist;
import org.ros.node.ConnectedNode;
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
    private static ConnectedNode connectedNode;

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

        LocalPlanner localPlanner=new LocalPlanner(connectedNode,tf);

        //set start and goal
        PoseStamped start=MsgFactory.messageFactory.newFromType(PoseStamped._TYPE);
        PoseStamped goal=MsgFactory.messageFactory.newFromType(PoseStamped._TYPE);
        start.getPose().getPosition().setX(1.0);
        start.getPose().getPosition().setY(1.0);
        start.getPose().getOrientation().setW(1);
        goal.getPose().getPosition().setX(3.0);
        goal.getPose().getPosition().setY(3.0);
        goal.getPose().getOrientation().setW(1);

        //make global plan
        GlobalPlanner globalPlanner=new GlobalPlanner();
        if(!globalPlanner.makePlan(start,goal,localPlanner.global_plan_)){
            logger.severe("unable to make a global plan");
            System.exit(-1);
        }

        //execute localPlanner
        NodeConfiguration configuration = NodeConfiguration.newPublic(IP.toString(),
                URI.create("http://localhost:11311"));
        final NodeMainExecutor runner= DefaultNodeMainExecutor.newDefault();
        String localPlannerId=new String("localPlanner_"+hostname);
        configuration.setNodeName(localPlannerId);
        //runner.execute(localPlanner,configuration);

        Twist cmd_vel= MsgFactory.messageFactory.newFromType(Twist._TYPE);
        Thread.sleep(5000);
        if(localPlanner.computeVelocityCommands(cmd_vel)){
            System.out.println("+++++++++++++++++++++Velocity computed!!");
        }else{
            System.out.println("---------------------Velocity computation failed!!");
        }

    }

    public   static   String StringFilter(String   str)   throws PatternSyntaxException {
        //clear special characters
        String regEx="[`~!@#$%^&*()+=|{}':;',\\[\\].<>/?~！@#￥%……&*（）——+|{}【】‘；：”“’。，、？-]";
        Pattern p   =   Pattern.compile(regEx);
        Matcher m   =   p.matcher(str);
        return   m.replaceAll("_").trim();
    }
}
