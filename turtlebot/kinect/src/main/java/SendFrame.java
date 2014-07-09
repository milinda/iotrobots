import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.MessageProperties;

import org.openkinect.freenect.*;
import com.jcraft.jzlib.*;

import java.io.*;
import java.io.IOException;
import java.lang.*;
import java.util.*;
import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

public class SendFrame {
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
            final String exchange_name = args[1];

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

            dev.startDepth(new DepthHandler() {
                int numFrame = 0;

                @Override
                public void onFrameReceived(FrameMode mode, ByteBuffer frame, int timestamp) {
                    // EVERY OTHER FRAME IS DISPLAYED
                    if (numFrame % 2 == 0) {
                        byte[] data = new byte[614400];
                        for (int i = 0; i < 614400; i++) data[i] = frame.get(i);


                        // COMPRESS DATA
                        int err;
                        int comprLen = 130000;
                        byte[] compr = new byte[comprLen];

                        Deflater deflater = null;
                        try {
                            deflater = new Deflater(JZlib.Z_BEST_SPEED);
                        } catch (GZIPException e) {
                            System.exit(0);
                        }

                        deflater.setInput(data);
                        deflater.setOutput(compr);

                        err = deflater.deflate(JZlib.Z_NO_FLUSH);
                        CHECK_ERR(deflater, err, "deflate");
                        if (deflater.avail_in != 0) {
                            System.out.println("deflate not greedy");
                            System.exit(1);
                        }

                        err = deflater.deflate(JZlib.Z_FINISH);
                        if (err != JZlib.Z_STREAM_END) {
                            System.out.println("deflate should report Z_STREAM_END");
                            System.exit(1);
                        }
                        err = deflater.end();

                        byte out[] = new byte[(int) deflater.total_out];
                        for (int i = 0; i < out.length; i++) {
                            out[i] = compr[i];
                        }
                        CHECK_ERR(deflater, err, "deflateEnd");

                        // PUBLISH COMPRESSED DATA
                        try {
                            channel.basicPublish(exchange_name, "", null, out);
                        } catch (IOException e) {
                            System.exit(0);
                        }
                    }
                    numFrame++;
                }
            });
        } catch (IOException e) {
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

