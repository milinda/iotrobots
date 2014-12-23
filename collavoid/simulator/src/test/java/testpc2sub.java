import cgl.iotrobots.collavoid.simulator.AgentNode;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import sensor_msgs.PointCloud2;

/**
 * Created by hjh on 12/1/14.
 */
public class testpc2sub {

    static PointCloud2 pc2;

    public static void main(String[] args) {
        AgentNode agentNode = new AgentNode("testTFsub");
        ConnectedNode node = agentNode.getNode();


        final Subscriber<PointCloud2> pc2sub = node.newSubscriber("/robot1/camera/depth/points", PointCloud2._TYPE);
        final Publisher<PointCloud2> pc2pub=node.newPublisher("testPc2",PointCloud2._TYPE);
       pc2sub.addMessageListener(new MessageListener<PointCloud2>() {
           @Override
           public void onNewMessage(PointCloud2 pointCloud2) {
               pc2pub.publish(pointCloud2);
//               for (int i = 0; i <pointCloud2.getFields().size() ; i++) {
//                   System.out.println(pointCloud2.getFields().get(i).getName());
//                   System.out.println(pointCloud2.getFields().get(i).getCount());
//                   System.out.println(pointCloud2.getFields().get(i).getDatatype());
//                   System.out.println(pointCloud2.getFields().get(i).getOffset());
//               }
//               System.out.println(pointCloud2.getPointStep());
//               System.out.println(pointCloud2.getIsBigendian());
//               System.out.println(pointCloud2.getData().readableBytes());

           }
       });
    }
}
