import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.MessageProperties;

import org.openkinect.freenect.*;
import com.jcraft.jzlib.*;

import java.io.*;
import java.lang.*;
import java.util.*;
import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

public class SendFrame {

    private static final String EXCHANGE_NAME = "kinect_frames";

    public static void main(String[] args) throws InterruptedException {
        try {

            ConnectionFactory factory = new ConnectionFactory();
            factory.setHost("156.56.93.102");
            Connection connection = factory.newConnection();
            final Channel channel = connection.createChannel();

            channel.exchangeDeclare(EXCHANGE_NAME, "fanout", false); //stops here...

            // DECLARATIONS
            Context ctx = null;
            Device dev = null;

            // INITIALIZE DEVICE
            ctx = Freenect.createContext();
            if (ctx.numDevices() > 0) {
                dev = ctx.openDevice(0);
            } else {
                System.err.println("No kinects detected.  Exiting.");
                System.exit(0);
            }
            // DISPLAY DEPTH VIDEO
            dev.startDepth(new DepthHandler() {
                int numFrame = 0;

                @Override
                public void onFrameReceived(FrameMode mode, ByteBuffer frame, int timestamp) {
                    if (numFrame % 2 == 0) {
                        byte[] data = new byte[614400];
                        for (int i = 0; i < 614400; i++) data[i] = frame.get(i);

                        Deflater deflater = new Deflater();
                        deflater.setInput(data);

                        ByteArrayOutputStream outputStream = new ByteArrayOutputStream(data.length);

                        deflater.finished();
                        byte[] buffer = new byte[1024];
                        while (!deflater.finished()) {
                            int count = deflater.deflate(data); // returns the generated code... index
                            outputStream.write(buffer, 0, count);
                        }
                        outputStream.close();
                        byte[] output = outputStream.toByteArray();
                        try {
                            channel.basicPublish(EXCHANGE_NAME, "kinect", null, compr);
                        } catch (IOException e) {
                            System.exit(0);
                        }

                    }
                    numFrame++;
                }
            });

            Thread.sleep(100000);
            channel.basicPublish(EXCHANGE_NAME, "", null, null);

            dev.stopDepth();

            channel.close();
            connection.close();

            // SHUT DOWN
            if (ctx != null) {
                if (dev != null) {
                    dev.close();
                }
            }
            ctx.shutdown();
        } catch (IOException e) {
            e.printStackTrace();
            System.exit(0);
        }
    }

    static void CHECK_ERR(ZStream z, int err, String msg) {
        if (err != JZlib.Z_OK) {
            if (z.msg != null) System.out.print(z.msg + " ");
            System.out.println(msg + " error: " + err);

            System.exit(1);
        }
    }
}
