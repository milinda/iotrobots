package cgl.iotrobots.collavoid.ROSAgent;

import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.InetAddress;
import java.net.URI;
import java.net.UnknownHostException;

import java.util.logging.Logger;
/**
 * Created by hjh on 11/9/14.
 */
public class runAgentAlone {

    private static Logger logger = Logger.getLogger("Agent");
    public static void main(String[] args)throws InterruptedException{


        String IP = null;
        String hostname = null;

        try {
            InetAddress ia = InetAddress.getLocalHost();
            hostname = ia.getHostName();//获取计算机主机名
            IP = ia.getHostAddress();//获取计算机IP
        } catch (UnknownHostException e) {
            e.printStackTrace();
        }

        logger.info("Standalone My name is: " + hostname);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        NodeConfiguration configuration = NodeConfiguration.newPublic(IP.toString(),
                URI.create("http://localhost:11311"));

        final NodeMainExecutor runner= DefaultNodeMainExecutor.newDefault();

        Agent agent=new Agent();

        configuration.setNodeName(hostname);

        runner.execute(agent,configuration);

    }
}
