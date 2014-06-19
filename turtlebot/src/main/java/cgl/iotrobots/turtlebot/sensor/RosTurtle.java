package cgl.iotrobots.turtlebot.sensor;

import geometry_msgs.Twist;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

import java.util.concurrent.BlockingQueue;

public class RosTurtle extends AbstractNodeMain {
    private boolean stop = false;

    private BlockingQueue<Velocity> velocities;

    public RosTurtle(BlockingQueue<Velocity> velocity) {
        this.velocities = velocity;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("/controller");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        final Publisher<Twist> publisher =
                connectedNode.newPublisher("/cmd_vel", Twist._TYPE);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException ie) {
            ie.printStackTrace();
        }

        // This CancellableLoop will be canceled automatically when the node shuts down.
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                Velocity v = velocities.take();

                Twist str = publisher.newMessage();

                if (v.getType() == Velocity.Type.LINEAR) {
                    str.getLinear().setX(v.getX());
                    str.getLinear().setY(v.getY());
                    str.getLinear().setZ(v.getZ());
                }

                if (v.getType() == Velocity.Type.ANGULAR) {
                    str.getAngular().setX(v.getX());
                    str.getAngular().setY(v.getY());
                    str.getAngular().setZ(v.getZ());
                }

                if (!stop) {
                    publisher.publish(str);
                }
            }
        });
    }

    public void onShutdown(Node node) {
        node.shutdown();
    }

    public void stop() {
        stop = true;
    }
}
