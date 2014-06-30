package cgl.iotrobots.turtlebot;

import cgl.iotrobots.turtlebot.commons.KinectMessageReceiver;
import cgl.iotrobots.turtlebot.commons.Motion;
import cgl.iotrobots.turtlebot.commons.RosTurtle;
import cgl.iotrobots.turtlebot.commons.Velocity;
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
import java.net.URI;
import java.net.URISyntaxException;
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
        this.turtle = new RosTurtle(velocities, "ui_controller");
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
        NodeConfiguration nodeConfiguration = null;
        try {
            nodeConfiguration = NodeConfiguration.newPublic("156.56.93.102", new URI("http://149.160.205.153:11311"));
            TurtleController turtleController = new TurtleController();
            turtleController.start(nodeConfiguration);
        } catch (URISyntaxException e) {
            e.printStackTrace();
        }
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
                        byte []uncompressed = unCompress((byte [])o);
                        BufferedImage image = getImage(uncompressed);
                        ui.setImage(image);
                        Motion motion = calculatePosition(uncompressed);
                        if (motion != null) {
                            velocities.add(motion);
                        }
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public static byte[] unCompress(byte []data) {
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
        return restored;
    }

    public Motion calculatePosition(byte[] depth_buf_) {
        double t_gamma[] = new double[2048];
        for (int p = 0; p < 2048; p++) {
            t_gamma[p] = 0.1236 * Math.tan(p / 2842.5 + 1.1863);
        }

        double max_z_ = 1.2;
        double min_y_ = .1;
        double max_y_ = .5;
        double max_x_ = .2;
        double min_x_ = -.2;
        double goal_z_ = .8;

        double z_scale_ = 1, x_scale_ = 20;

        int height_ = 480;
        int width_ = 640;
        double minDistance = -10;
        double scaleFactor = .0021;
        double x, y, z;
        double totX = 0, totY = 0, totZ = 0;
        int k = 0, n = 0;

        double cx = 320.0; // center of projection
        double cy = 240.0; // center of projection
        double fx = 600.0; // focal length in pixels
        double fy = 600.0; // focal length in pixels

        for (int v = 0; v < height_; ++v) {
            for (int u = 0; u < width_; ++u) {
                int lo = depth_buf_[k] & 0xFF;
                int hi = depth_buf_[k + 1] & 0xFF;
                int disp = hi << 8 | lo;

                double d = t_gamma[disp];
                if (d <= 0.0)
                    continue;
                // Fill in XYZ
//                z = d;
//                x = (u - width_ / 2) * (z + minDistance) * scaleFactor;
//                y = (v - height_ / 2) * (z + minDistance) * scaleFactor;
                x = (u - cx) * d / fx;
                y = (v - cy) * d / fy;
                z = d;
                //System.out.format("x %f, y %f, z %f\n", x, y, z);

                if (y > min_y_ && y < max_y_ && x < max_x_ && x > min_x_ && z < max_z_) {
                    //Add the point to the totals
                    totX += x;
                    totY += y;
                    totZ += z;
                    n++;
                }
                k += 2;
            }
        }

        if (n > 4000) {
            totX /= n;
            totY /= n;
            totZ /= n;
            System.out.format("x %f, y %f, z %f\n", totX, totY, totZ);
            if (Math.abs(totZ  -max_z_) < 0.01) {
                //System.out.println("No valid points detected, stopping the robot");
                return new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, 0));
            }

            // System.out.format("Centroid at %f %f %f with %d points", totX, totY, totZ, n);
            if (totX * x_scale_ >= .1) {
                return new Motion(new Velocity((totZ - goal_z_) * z_scale_, 0, 0), new Velocity(0, 0, totX * x_scale_));
            } else {
                return new Motion(new Velocity((totZ - goal_z_) * z_scale_, 0, 0), new Velocity(0, 0, 0));
            }

        } else {
            // System.out.println("No valid points detected, stopping the robot");
            return new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, 0));
        }
    }

    public static BufferedImage getImage(byte []restored) {
        double t_gamma[] = new double[1024];
        for (int p = 0; p < 1024; p++) {
            t_gamma[p] = 100 * 0.1236 * Math.tan(p / 2842.5 + 1.1863);
        }

        BufferedImage im = new BufferedImage(640, 480, BufferedImage.TYPE_INT_RGB);
        ByteBuffer frame = ByteBuffer.allocate(614400);

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
