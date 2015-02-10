package cgl.iotrobots.turtlebot;

import cgl.iotrobots.turtlebot.commons.KinectMessageReceiver;
import cgl.iotrobots.turtlebot.commons.Motion;
import cgl.iotrobots.turtlebot.commons.Velocity;
import cgl.iotrobots.turtlebot.commons.Compressor;
//import com.google.common.collect.Lists;
import com.jcraft.jzlib.Inflater;
import com.jcraft.jzlib.JZlib;
import com.jcraft.jzlib.ZStream;
//import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
//import java.nio.ByteBuffer;
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
        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(turtle, configuration);

        messageReceiver.start();


        EventHandler eventHandler = new EventHandler(this);

        TurtleBotUIImpl ui = new TurtleBotUIImpl(eventHandler);
        Thread t = new Thread(new UIUpdater(messages, ui));
        t.start();
        ui.setVisible(true);

        try {
            test();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void test() throws InterruptedException {
        for (int i = 0; i < 10; i++) {
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
        NodeConfiguration nodeConfiguration;
        try {
            nodeConfiguration = NodeConfiguration.newPublic("156.56.93.59", new URI("http://156.56.95.50:11311"));
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
                        Compressor compressor = new Compressor();
                        int[] dist = compressor.unCompr((byte[]) o);
                        BufferedImage image = getImage(dist);
                        ui.setImage(image);
                        //detect(ui, dist);
                        Motion motion = calculatePosition(dist);
                        if (motion != null) {
                            velocities.add(motion);
                        }
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    /*public static void detect(TurtleBotUIImpl ui, int[] dist) {
        for (int i = 0; i < dist.length; i++) {
            if (dist[i] < 750) {
                ui.setObstacle(true);
                return;
            }
        }
        ui.setObstacle(false);
    }*/

    public Motion calculatePosition(int[] dist) {

        double max_z_ = 1.0;
        double min_y_ = .1;
        double max_y_ = .5;
        double max_x_ = .2;
        double min_x_ = -.2;
        double goal_z_ = .7;

        double z_scale_ = 1, x_scale_ = 5.0;

        double x, y, z;
        double totX = 0, totY = 0, totZ = 0;
        int n = 0;

        double cx = 320.0; // center of projection
        double cy = 240.0; // center of projection
        double fx = 600.0; // focal length in pixels
        double fy = 600.0; // focal length in pixels

        int u = 0,v = 0;
        for(int i=0; i<307200; i++) {
                v = (int) Math.floor((double) i / 640);
                u = i - 640 * v;
                //double d = (double) (dist[i]) / 1000;
                double d = ((double) dist[i]) / 1000;
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
                //u++;
                //if(i % 640 == 0) v++;
        }
        if (n > 4000) {
            totX /= n;
            //totY /= n;
            totZ /= n;
            //System.out.format("x %f, y %f, z %f\n", totX, totY, totZ);
            if (Math.abs(totZ  -max_z_) < 0.01) {
                //System.out.println("No valid points detected, stopping the robot");
                return new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, 0));
            }

            //System.out.format("Centroid at %f %f %f with %d points\n", totX, totY, totZ, n);
            if (Math.abs(totX * x_scale_) >= .1) {
                return new Motion(new Velocity((totZ - goal_z_) * z_scale_, 0, 0), new Velocity(0, 0, (-1)*totX * x_scale_));
            } else {
                return new Motion(new Velocity((totZ - goal_z_) * z_scale_, 0, 0), new Velocity(0, 0, 0));
            }

        } else {
            // System.out.println("No valid points detected, stopping the robot");
            return new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, 0));
        }
    }

    public static BufferedImage getImage(int []dist) {

        BufferedImage im = new BufferedImage(640, 480, BufferedImage.TYPE_INT_RGB);

        int r=0,b=0,g=0,x,y;
        for(int i=0; i<307200; i++) {
            if(dist[i] == 0) {
                b = 0;
                r = 0;
                g = 0;
            } else {
                if (dist[i] >= 326 && dist[i] < 1443) {
                    b = 255;
                    r = 0;
                    g = 255;
                } else if (dist[i] >= 1443 && dist[i] <= 2500) {
                    b = 255;
                    r = 0;
                    g = 150;
                } else if (dist[i] > 2500 && dist[i] <= 4185){
                    b = 255;
                    r = 0;
                    g = 50;
                }
            }

            y=(int)Math.floor((double)i/640);
            x=i-640*y;
            int pixel = (0xFF) << 24
                    | (b & 0xFF) << 16
                    | (g & 0xFF) << 8
                    | (r & 0xFF) << 0;
            im.setRGB(x, y, pixel);
        }

        return im;
    }

}
