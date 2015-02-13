package cgl.iotrobots.slam.utils;

import cgl.iotrobots.slam.utils.MessageFilter;
import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import nav_msgs.Odometry;
import org.apache.commons.lang3.tuple.Pair;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import sensor_msgs.LaserScan;
import tf2_msgs.TFMessage;

import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

/**
* Created by supun on 2/10/15.
*/
public class RosTurtle extends AbstractNodeMain {
    private String name = "/ts_controller";

    private BlockingQueue<Pair<Odometry, LaserScan>> queue = new ArrayBlockingQueue<Pair<Odometry, LaserScan>>(64);

    private MessageFilter filter = new MessageFilter(10, queue);

    public RosTurtle() {
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("/" + name);
    }

    public BlockingQueue<Pair<Odometry, LaserScan>> getQueue() {
        return queue;
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        System.out.println("Starting....");
        final Subscriber<Odometry> odometrySubscriber =
                connectedNode.newSubscriber("/odom", Odometry._TYPE);

        final Subscriber<LaserScan> laserScanSubscriber =
                connectedNode.newSubscriber("/scan", LaserScan._TYPE);

        final Publisher<TFMessage> transformPublisher = connectedNode.newPublisher("/tf", TFMessage._TYPE);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException ie) {
            ie.printStackTrace();
        }

        odometrySubscriber.addMessageListener(new MessageListener<Odometry>() {
            @Override
            public void onNewMessage(Odometry odometry) {
                try {
                    filter.addOdometry(odometry);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });

        laserScanSubscriber.addMessageListener(new MessageListener<LaserScan>() {
            @Override
            public void onNewMessage(LaserScan laserScanMsg) {
                try {
                    filter.addLaserScan(laserScanMsg);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                sendTransform(connectedNode, transformPublisher, "map", "odom", System.currentTimeMillis() + 5000, 0, 0, 0, 0, 0, 0, 1);
                Thread.sleep(100);
            }
        });
    }

    public void sendTransform(ConnectedNode node, Publisher<TFMessage> pub,
                              String parentFrame, String childFrame,
                              long t,
                              double v_x, double v_y, double v_z,
                              double q_x, double q_y, double q_z, double q_w) {

        TransformStamped txMsg = node.getTopicMessageFactory().newFromType(TransformStamped._TYPE);
        txMsg.getHeader().setStamp(org.ros.message.Time.fromNano(t));
        txMsg.getHeader().setFrameId(parentFrame);
        txMsg.setChildFrameId(childFrame);

        Vector3 vector3 = node.getTopicMessageFactory().newFromType(Vector3._TYPE);
        vector3.setY(v_y);
        vector3.setX(v_x);
        vector3.setZ(v_z);
        txMsg.getTransform().setTranslation(vector3);

        Quaternion quaternion = node.getTopicMessageFactory().newFromType(Quaternion._TYPE);
        quaternion.setX(q_x);
        quaternion.setY(q_y);
        quaternion.setZ(q_z);
        quaternion.setW(q_w);
        txMsg.getTransform().setRotation(quaternion);

        TFMessage msg = node.getTopicMessageFactory().newFromType(TFMessage._TYPE);
        msg.setTransforms(new ArrayList<TransformStamped>(1));
        msg.getTransforms().add(txMsg);

        Transform transform = node.getTopicMessageFactory().newFromType(Transform._TYPE);
        transform.setRotation(quaternion);

        pub.publish(msg);
    }

    public void onShutdown(Node node) {
        node.shutdown();
    }
}
