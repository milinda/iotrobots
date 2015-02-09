package cgl.iotrobots.turtlebot;

import cgl.iotrobots.turtlebot.commons.Motion;
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

    private BlockingQueue<Motion> velocities;

    private String name = "/cn";

    public RosTurtle(BlockingQueue<Motion> velocities) {
        this.velocities = velocities;
    }

    public RosTurtle(BlockingQueue<Motion> velocity, String name) {
        this.velocities = velocity;
        this.name = name;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("/" + name);
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        System.out.println("Starting....");
        final Publisher<Twist> publisher =
//                connectedNode.newPublisher("/cmd_vel_mux/input/teleop", Twist._TYPE);
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
                //System.out.println("loop..");
                Motion m = velocities.take();
                Twist str = publisher.newMessage();
                //System.out.println("Linear...");
                str.getLinear().setX(m.getLinear().getX());
                str.getLinear().setY(m.getLinear().getY());
                str.getLinear().setZ(m.getLinear().getZ());

                str.getAngular().setX(m.getAngular().getX());
                str.getAngular().setY(m.getAngular().getY());
                str.getAngular().setZ(m.getAngular().getZ());

                publisher.publish(str);
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
