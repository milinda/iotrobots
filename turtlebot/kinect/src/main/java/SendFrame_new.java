import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.MessageProperties;

import org.openkinect.freenect.*;
import org.xerial.snappy.Snappy;

import java.io.*;
import java.io.IOException;
import java.lang.*;
import java.util.*;
import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

public class SendFrame_new {
    // DECLARATIONS
    private static Context ctx = null;
    private static Device dev = null;

    public static void main(String[] args) throws InterruptedException {
        try {
            // ESTABLISH RABBITMQ CONNECTION
            ConnectionFactory factory = new ConnectionFactory();
            factory.setHost(args[0]);
            final Connection connection = factory.newConnection();
            final Channel channel = connection.createChannel();
            final String exchange_name = "kinect_frames";
            // CREATE EXCHANGE
            channel.exchangeDeclare(exchange_name, "fanout");

            addShutDownHook(channel, connection);

            // INITIALIZE DEVICE
            ctx = Freenect.createContext();
            if (ctx.numDevices() > 0) {
                dev = ctx.openDevice(0);
            } else {
                System.err.println("No kinects detected.  Exiting.");
                System.exit(0);
            }

            final int t_gamma[] = new int[1024];
            final byte inverted[] = new byte[1024];
            for (int i = 0; i < 1024; i++) {
                t_gamma[i] = (int) (1000 * 0.1236 * Math.tan(i / 2842.5 + 1.1863));
                inverted[i] = (byte) (90300 / t_gamma[i] - 21.575);
            }

            // START DEPTH VIDEO
            dev.startDepth(new DepthHandler() {
                int numFrame = 0;

                @Override
                public void onFrameReceived(FrameMode mode, ByteBuffer frame, int timestamp) {
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

                            // COMPRESS DATA
                       
                            // COMPRESS AND PUBLISH COMPRESSED DATA
                            try {
                            	byte[] compr = Snappy.compress(data);
                                channel.basicPublish(exchange_name, "", null, compr);
                            } catch (IOException e) {
                                System.exit(0);
                            }
                }
                }
                );
            }catch(IOException e){
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

