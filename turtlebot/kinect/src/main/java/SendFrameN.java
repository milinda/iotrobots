import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;

import org.openkinect.freenect.*;
import org.xerial.snappy.Snappy;

import java.io.*;
import java.lang.*;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

public class SendFrameN {
    private static Context ctx = null;
    private static Device dev = null;
    private static int counter = 0;
    private static int c2 = 0;
    private static int[] t_gamma;
    private static byte[] inverted;
    private static Channel channel;
    private static final String exchange_name = "kinect_frames";

    public static void main(String[] args) {
        try {
            ConnectionFactory factory = new ConnectionFactory();
            factory.setHost(args[0]);
            final Connection connection = factory.newConnection();
            channel = connection.createChannel();
            channel.exchangeDeclare(exchange_name, "fanout");
            addShutDownHook(channel, connection);

            t_gamma = new int[1024];
            inverted = new byte[1024];
            for (int i = 0; i < 1024; i++) {
                t_gamma[i] = (int) (1000 * 0.1236 * Math.tan(i / 2842.5 + 1.1863));
                inverted[i] = (byte) (90300 / t_gamma[i] - 21.575);
            }

//            startKinect();
            readFromFile();
        } catch (Exception e) {
            e.printStackTrace();
            System.exit(0);
        }
    }

    private static class KinectDepthHandler implements DepthHandler {
        @Override
        public void onFrameReceived(FrameMode frameMode, ByteBuffer byteBuffer, int i) {
            byte []compressed = compress(byteBuffer, inverted);
            send(channel, exchange_name, compressed);
        }
    }

    private static void startKinect() throws IOException {
        // INITIALIZE DEVICE
        ctx = Freenect.createContext();
        if (ctx.numDevices() > 0) {
            dev = ctx.openDevice(0);
        } else {
            System.err.println("No kinects detected.  Exiting.");
            System.exit(0);
        }
        // START DEPTH VIDEO
        dev.startDepth(new KinectDepthHandler());
    }

    private static void readFromFile() throws IOException, InterruptedException {
        for (int i = 0; i < 1500; i++) {
            RandomAccessFile aFile = new RandomAccessFile(
                    "/home/supun/dev/projects/kinect/raw/frame_" + counter++, "r");
            FileChannel inChannel = aFile.getChannel();
            ByteBuffer buffer = ByteBuffer.allocate(614400);
            inChannel.read(buffer);
            buffer.flip();
            byte[] data = compress(buffer, inverted);
            send(channel, exchange_name, data);
            Thread.sleep(33);
        }
    }

    private static void saveCompressed(byte data[]) {
        File file = new File("/home/supun/dev/projects/kinect/compressed/frame_" + c2++);
        try {
            FileOutputStream wChannel = new FileOutputStream(file, false);
            wChannel.write(data);
            wChannel.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void saveRaw(ByteBuffer frame) {
        File file = new File("/home/supun/dev/projects/kinect/raw/frame_" + counter++);
        try {
            FileChannel wChannel = new FileOutputStream(file, false).getChannel();
            wChannel.write(frame);
            frame.flip();
            wChannel.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static byte[] compress(ByteBuffer frame, byte[] inverted) {
        byte[] data = new byte[307200];
        int p = 0;
        for (int i = 0; i < 614400; i += 2) {
            int lo = frame.get(i) & 0xFF;
            int hi = frame.get(i + 1) & 0xFF;
            int disp = hi << 8 | lo;
            if (disp > 60 && disp < 1012) {
                data[p] = inverted[disp];
            } else {
                data[p] = 0;
            }
            p++;
        }
        try {
            data = Snappy.compress(data);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return data;
    }

    private static void send(Channel channel, String exchange_name, byte[] data) {
        try {
            channel.basicPublish(exchange_name, "", null, data);
        } catch (IOException e) {
            e.printStackTrace();
            System.exit(0);
        }
    }

    static void addShutDownHook(final Channel channel, final Connection connection) {
        // CLOSE CLEANLY ON EXIT
        Runtime.getRuntime().addShutdownHook(new Thread() {
            @Override
            public void run() {
                dev.stopDepth();
                try {
                    channel.close();
                    connection.close();
                } catch (IOException e) {
                    System.exit(0);
                }
                // SHUT DOWN
                if (ctx != null) {
                    if (dev != null) {
                        dev.close();
                    }
                }
                ctx.shutdown();
            }
        });
    }
}

