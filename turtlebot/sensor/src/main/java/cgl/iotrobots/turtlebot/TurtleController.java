package cgl.iotrobots.turtlebot;

import cgl.iotrobots.turtlebot.commons.KinectMessageReceiver;
import cgl.iotrobots.turtlebot.commons.Motion;
import cgl.iotrobots.turtlebot.commons.RosTurtle;
import cgl.iotrobots.turtlebot.commons.Velocity;
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

    private KinectMessageReceiver messageReceiver;

    private BlockingQueue messages = new LinkedBlockingQueue();

    public TurtleController() {
        this.turtle = new RosTurtle(velocities);
        this.messageReceiver = new KinectMessageReceiver(messages, "kinect_controller", null, null, "amqp://localhost:5672");

        this.messageReceiver.setExchangeName("kinect_frames");
        this.messageReceiver.setRoutingKey("kinect");
    }

    public void start(NodeConfiguration configuration) {
        Preconditions.checkState(turtle != null);
        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(turtle, configuration);

        messageReceiver.start();
    }

    public void test() throws InterruptedException {
        for (int i = 0 ;i < 2; i++) {
            velocities.add(new Motion(new Velocity(-.1, 0, 0), new Velocity(0, 0, 0)));
            Thread.sleep(100);
        }
        Thread.sleep(100);
    }

    public void setMotion(Motion motion) {
        try {
            velocities.put(motion);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
