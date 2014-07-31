package cgl.iotrobots.turtlebot;

import cgl.iotrobots.turtlebot.commons.Motion;
import com.google.common.base.Preconditions;
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
        this.turtle = new RosTurtle(velocities, "sensor_controller");
    }

    public void start(NodeConfiguration configuration) {
        Preconditions.checkState(turtle != null);
        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(turtle, configuration);
    }

    public void setMotion(Motion motion) {
        try {
            velocities.put(motion);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void shutDown() {
        if (nodeMainExecutor != null) {
            nodeMainExecutor.shutdown();
        }
    }
}
