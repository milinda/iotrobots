package cgl.iotrobots.turtlebot;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class TurtleController {
    private RosTurtle turtle;

    private BlockingQueue<Motion> velocities = new LinkedBlockingQueue<Motion>();

    private NodeMainExecutor nodeMainExecutor;

    public TurtleController() {
        this.turtle = new RosTurtle(velocities);
    }

    public void start(NodeConfiguration configuration) {
        Preconditions.checkState(turtle != null);
        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(turtle, configuration);
    }

    public void test() throws InterruptedException {
        for (int i = 0 ;i < 2; i++) {
            velocities.add(new Motion(new Velocity(-.1, 0, 0, Velocity.Type.LINEAR), new Velocity(0, 0, 0, Velocity.Type.ANGULAR)));
            Thread.sleep(100);
        }
//        velocities.add(new Motion(new Velocity(0, 0, 0, Velocity.Type.LINEAR), new Velocity(0, 0, 0, Velocity.Type.ANGULAR)));
//        velocities.add(new Motion(new Velocity(0, 0, 0, Velocity.Type.LINEAR), new Velocity(0, 0, 0, Velocity.Type.ANGULAR)));
//        velocities.add(new Motion(new Velocity(0, 0, 0, Velocity.Type.LINEAR), new Velocity(0, 0, 0, Velocity.Type.ANGULAR)));
        Thread.sleep(100);
    }

    public static void main(String[] args) throws InterruptedException {
        // register with ros_java
        CommandLineLoader loader = new CommandLineLoader(Lists.newArrayList(args));
        NodeConfiguration nodeConfiguration = loader.build();

        TurtleController turtleController = new TurtleController();
        turtleController.start(nodeConfiguration);

        turtleController.test();
    }
}
