import geometry_msgs.Twist;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.*;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import javax.vecmath.Vector3d;
import java.net.URI;

/**
 * Created by hjh on 11/27/14.
 */
public class VelCmdPub {

    public static class VelCmdPubNode extends AbstractNodeMain {
        Twist velcmd;
        int seq;

        @Override
        public void onStart(ConnectedNode connectedNode) {
            final Publisher<Twist> velCmdPub = connectedNode.newPublisher("robot0/cmd_vel", Twist._TYPE);

            connectedNode.executeCancellableLoop(new CancellableLoop() {
                @Override
                protected void setup() {
                    velcmd=velCmdPub.newMessage();
                    velcmd.getLinear().setX(0.2);
                    velcmd.getLinear().setY(-0.2);
                    seq=0;
                }

                @Override
                protected void loop() throws InterruptedException {
                    velCmdPub.publish(velcmd);
                    seq++;
                    if(seq==10){
                        velcmd.getAngular().setZ(0.2);
                    }
                    Thread.sleep(100);
                }
            });

        }

        @Override
        public GraphName getDefaultNodeName() {
            return GraphName.of("robot");
        }

    }

    public static void main(String[] args){
        //execute localPlanner
        NodeConfiguration configuration = NodeConfiguration.newPublic("localhost",
                URI.create("http://localhost:11311"));
        final NodeMainExecutor runner = DefaultNodeMainExecutor.newDefault();
        configuration.setNodeName("velpub");

        VelCmdPubNode pubNode=new VelCmdPubNode();
        runner.execute(pubNode,configuration);
    }
}
