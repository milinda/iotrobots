import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.QueueingConsumer;

import com.jcraft.jzlib.*;
import org.xerial.snappy.Snappy;

import java.io.*;
import java.lang.*;
import java.awt.image.BufferedImage;

public class RecvFrameN {
    public static void main(String[] args) throws InterruptedException, IOException {
        final ImageUI ui = new ImageUI();
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                ui.show();
            }
        });
        t.start();

        // RABBITMQ DECLARATIONS
        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost(args[0]);
        final Connection connection = factory.newConnection();
        final Channel channel = connection.createChannel();
        String exchange_name = "turtle_kinect";

        // BIND EXCHANGE TO QUEUE
        channel.exchangeDeclare(exchange_name, "fanout");
        String queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, exchange_name, "");

        QueueingConsumer consumer = new QueueingConsumer(channel);
        channel.basicConsume(queueName, true, consumer);

        // CLEANLY SHUTDOWN
        Runtime.getRuntime().addShutdownHook(new Thread() {
            @Override
            public void run() {
                System.out.println("exitting");
                try {
                    channel.close();
                } catch (IOException e) {
                    System.exit(0);
                }
            }
        });

        BufferedImage im = new BufferedImage(640, 480, BufferedImage.TYPE_INT_RGB);
        QueueingConsumer.Delivery delivery = null;
        System.out.println("Waiting for delivery");

        do {
            // POP DATA OF QUEUE
            delivery = consumer.nextDelivery();
            int err;
            byte[] data = delivery.getBody();
            byte[] inverted = Snappy.uncompress(data);


            // COLOR DISTANCE DATA
            int[] restored = new int[307200];
            int r = 0, b = 0, g = 0, x, y;
            for (int i = 0; i < 307200; i++) {
                restored[i] = 0 | (inverted[i] & 0xFF);
                if (restored[i] == 0) {
                    b = 0;
                    r = 0;
                    g = 0;
                } else {
                    int dist = (int) (90300 / (restored[i] + 21.575));
                    if (dist >= 326 && dist < 1443) {
                        b = 255;
                        r = 0;
                        g = 255;
                    } else if (dist >= 1443 && dist <= 2500) {
                        b = 255;
                        r = 0;
                        g = 150;
                    } else if (dist > 2500 && dist <= 4185) {
                        b = 255;
                        r = 0;
                        g = 50;
                    }
                }

                y = (int) Math.floor((double) i / 640);
                x = i - 640 * y;
                int pixel = (0xFF) << 24
                        | (b & 0xFF) << 16
                        | (g & 0xFF) << 8
                        | (r & 0xFF) << 0;
                im.setRGB(x, y, pixel);
            }
            ui.setImage(im);
            ui.repaint();
        } while (true);
    }

    static void CHECK_ERR(ZStream z, int err, String msg) {
        if (err != JZlib.Z_OK) {
            if (z.msg != null) System.out.print(z.msg + " ");
            System.out.println(msg + " error: " + err);

            System.exit(1);
        }
    }
}


