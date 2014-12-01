import geometry_msgs.Twist;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.*;
import org.ros.node.topic.Subscriber;
import sensor_msgs.PointCloud2;

import javax.vecmath.Vector3d;
import java.net.InetAddress;
import java.net.URI;
import java.net.UnknownHostException;

/**
 * Created by hjh on 11/24/14.
 */
public class CmdVelSub {
    private String nodeName;
    private double vl;
    private double vr;
    private double wheelDistance;

    public class VelListener extends AbstractNodeMain {

        @Override
        public void onStart(ConnectedNode connectedNode) {
            final Subscriber<Twist> velSub = connectedNode.newSubscriber(connectedNode.getName().toString() + "/cmd_vel", Twist._TYPE);
            velSub.addMessageListener(new MessageListener<Twist>() {
                @Override
                public void onNewMessage(Twist msg) {
                    double v, w;
                    Vector3d vel = new Vector3d(msg.getLinear().getX(), msg.getLinear().getY(), msg.getLinear().getZ());
                    v = vel.length();
                    w = msg.getAngular().getZ();
                    vl = (2 * v - w * wheelDistance) / 2;
                    vr = (2 * v + w * wheelDistance) / 2;
                }
            });

        }

        @Override
        public GraphName getDefaultNodeName() {
            return GraphName.of("robot");
        }

    }

    public CmdVelSub(double dist, String nodeName) {
        this.nodeName = nodeName;
        this.wheelDistance = dist;
        initalize();
    }


    public double getVl() {
        return vl;
    }

    public double getVr() {
        return vr;
    }

    public void initalize() {

        String IP = null;
        String hostname = null;

        try {
            InetAddress ia = InetAddress.getLocalHost();
            hostname = ia.getHostName();
            IP = ia.getHostAddress();
        } catch (UnknownHostException e) {
            e.printStackTrace();
        }

        VelListener velListener = new VelListener();

        //execute localPlanner
        NodeConfiguration configuration = NodeConfiguration.newPublic(IP.toString(),
                URI.create("http://localhost:11311"));
        final NodeMainExecutor runner = DefaultNodeMainExecutor.newDefault();

        configuration.setNodeName(nodeName);
        runner.execute(velListener, configuration);
    }

}

