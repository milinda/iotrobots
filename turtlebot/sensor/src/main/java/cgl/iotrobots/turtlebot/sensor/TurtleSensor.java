package cgl.iotrobots.turtlebot.sensor;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class TurtleSensor {
    private RosTurtle turtle = null;

    private NodeMainExecutor nodeMainExecutor;

    private BlockingQueue<Velocity> velocities = new LinkedBlockingQueue<Velocity>();

    public TurtleSensor() {
        turtle = new RosTurtle(velocities);
    }

    public void start(NodeConfiguration nodeConfiguration) {
        Preconditions.checkState(turtle != null);
        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(turtle, nodeConfiguration);
    }

    public static void main(String[] argv) throws Exception {
        // register with ros_java
        CommandLineLoader loader = new CommandLineLoader(Lists.newArrayList(argv));
        NodeConfiguration nodeConfiguration = loader.build();
        final TurtleSensor sensor = new TurtleSensor();

        sensor.start(nodeConfiguration);

        Runtime.getRuntime().addShutdownHook(new Thread() {
            @Override
            public void run() {
                sensor.stop();
            }
        });
    }

    private void stop() {
        System.out.println("Shutting down sensor.....");
        turtle.stop();
        nodeMainExecutor.shutdown();
    }
}
