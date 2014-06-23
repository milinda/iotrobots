package cgl.iotrobots.turtlebot;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.jcraft.jzlib.Inflater;
import com.jcraft.jzlib.JZlib;
import com.jcraft.jzlib.ZStream;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;
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


        EventHandler eventHandler = new EventHandler(this);

        TurtleBotUIImpl ui = new TurtleBotUIImpl(eventHandler);
        Thread t = new Thread(new UIUpdater(messages, ui));
        t.start();
        ui.setVisible(true);
    }

    public void test() throws InterruptedException {
        for (int i = 0 ;i < 2; i++) {
            velocities.add(new Motion(new Velocity(-.1, 0, 0), new Velocity(0, 0, 0)));
            Thread.sleep(100);
        }
        Thread.sleep(100);
    }

    public void addVelocity(Motion motion) {
        try {
            velocities.put(motion);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) throws InterruptedException {
        // register with ros_java
        CommandLineLoader loader = new CommandLineLoader(Lists.newArrayList(args));
        NodeConfiguration nodeConfiguration = loader.build();

        TurtleController turtleController = new TurtleController();
        turtleController.start(nodeConfiguration);


    }

    public class UIUpdater implements Runnable {
        private BlockingQueue messages;

        private TurtleBotUIImpl ui;

        public UIUpdater(BlockingQueue messages, TurtleBotUIImpl ui) {
            this.messages = messages;
            this.ui = ui;
        }

        @Override
        public void run() {
            while (true) {
                try {
                    Object o = messages.take();
                    if (o instanceof byte[]) {
                        BufferedImage image = getImage((byte[]) o);
                        ui.setImage(image);
                    }

                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public static BufferedImage getImage(byte []data) {
        double t_gamma[] = new double[1024];
        for (int p = 0; p < 1024; p++) {
            t_gamma[p] = 100 * 0.1236 * Math.tan(p / 2842.5 + 1.1863);
        }

        BufferedImage im = new BufferedImage(640, 480, BufferedImage.TYPE_INT_RGB);
        ByteBuffer frame = ByteBuffer.allocate(614400);
        System.out.println("Waiting for delivery");

        int err;
        byte[] restored = new byte[614400];

        Inflater inflater = new Inflater();

        inflater.setInput(data);

        while (true) {
            inflater.setOutput(restored);
            err = inflater.inflate(JZlib.Z_NO_FLUSH);
            if (err == JZlib.Z_STREAM_END) break;
            CHECK_ERR(inflater, err, "inflate large");
        }

        err = inflater.end();
        CHECK_ERR(inflater, err, "inflateEnd");

        for (int i = 0; i < 614400; i++) frame.put(i, restored[i]);

        int r, b, g, x, y;
        for (int i = 0; i < 614400; i += 2) {
            int lo = frame.get(i) & 0xFF;
            int hi = frame.get(i + 1) & 0xFF;
            int disp = hi << 8 | lo;
            double dist = t_gamma[disp];
            if (dist >= 40 && dist < 150) {
                b = 255;
                r = 0;
                g = (int) (255 - ((dist - 40) / 109 * 255));
            } else if (dist >= 150 && dist <= 250) {
                dist = ((dist - 150) / 100 * 255);
                b = (int) (255 - dist);
                r = (int) (dist);
                g = 0;
            } else if (dist > 250 && dist <= 500) {
                dist = (dist - 251) / 249 * 255;
                b = 0;
                r = (int) (255 - dist);
                g = (int) (dist);
            } else if (disp == 1023) {
                b = 0;
                r = 0;
                g = 0;
            } else {
                dist = (dist - 501) / t_gamma[1022] * 255;
                b = 20;
                r = 0;
                g = (int) (255 - dist);
            }

            y = (int) Math.floor((double) i / 2 / 640);
            x = i / 2 - 640 * y;
            int pixel = (0xFF) << 24
                    | (b & 0xFF) << 16
                    | (g & 0xFF) << 8
                    | (r & 0xFF) << 0;
            im.setRGB(x, y, pixel);
        }

        return im;
    }

    static void CHECK_ERR(ZStream z, int err, String msg) {
        if (err != JZlib.Z_OK) {
            if (z.msg != null) System.out.print(z.msg + " ");
            System.out.println(msg + " error: " + err);

            System.exit(1);
        }
    }
}
