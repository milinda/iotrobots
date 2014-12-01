import org.jgrapht.graph.ListenableDirectedGraph;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.rosjava.tf.Transform;
import org.ros.rosjava.tf.TransformBuffer;
import org.ros.rosjava.tf.pubsub.TransformListener;
import sensor_msgs.PointCloud2;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Created by hjh on 11/30/14.
 */
public class testTF {
    static TransformListener tfl;
    static PointCloud2 pc2;

    public static void main(String[] args){
        AgentNode agentNode=new AgentNode("testTF");
        ConnectedNode node=agentNode.getNode();

        List<Point3d> scan=new ArrayList<Point3d>();
        for (int i = 0; i <100 ; i++) {
            Point3d pt=new Point3d(Math.random(),Math.random(),Math.random());
            scan.add(pt);
        }
        pc2=node.getTopicMessageFactory().newFromType(PointCloud2._TYPE);
        utils.toPointCloud2(pc2,scan);
        final Publisher<PointCloud2> pc2pub=node.newPublisher("testPc2",PointCloud2._TYPE);
        node.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                System.out.println(pc2.getData().capacity());
                pc2pub.publish(pc2);
                Thread.sleep(1000);
            }
        });


    }
}
