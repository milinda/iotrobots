package cgl.iotrobots.sim;

import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.slam.core.app.GFSMap;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.streaming.Utils;
import cgl.iotrobots.utils.rabbitmq.*;
import com.esotericsoftware.kryo.Kryo;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class FileBasedDistributedSimulator {
    RabbitMQSender dataSender;
    RabbitMQSender controlSender;
    RabbitMQReceiver receiver;
    RabbitMQReceiver bestReceiver;
//    private String url = "amqp://localhost:5672";
    BufferedReader br = null;
    Kryo kryo = new Kryo();
//    private String url = "amqp://149.165.159.3:5672";
    public FileBasedDistributedSimulator(String url) {
        try {
            dataSender = new RabbitMQSender(url, "simbard_laser");
            controlSender = new RabbitMQSender(url, "simbard_control");
            receiver = new RabbitMQReceiver(url, "simbard_map");
            bestReceiver = new RabbitMQReceiver(url, "simbard_best");

            dataSender.open();
            controlSender.open();
            receiver.listen(new MapReceiver());
            bestReceiver.listen(new BestParticleReceiver());

            br = new BufferedReader(new FileReader("out.txt"));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void start() throws InterruptedException {
        byte[] body = "start".getBytes();
        Map<String, Object> props = new HashMap<String, Object>();
        props.put("time", System.currentTimeMillis());
        props.put(TransportConstants.SENSOR_ID, System.currentTimeMillis());
        Message message = new Message(body, props);
        try {
            controlSender.send(message, "test.test.control");
            Thread.sleep(1000);
        } catch (Exception e) {
            e.printStackTrace();
        }

        Thread t = new Thread(new SendWorker());
        t.start();
        t.join();
    }

    public static void main(String[] args) throws InterruptedException {
        FileBasedDistributedSimulator fileBasedSimulator = new FileBasedDistributedSimulator(args[0]);
        fileBasedSimulator.start();
    }

    private class SendWorker implements Runnable {
        @Override
        public void run() {
            while (true) {
                String line;
                try {
                    line = br.readLine();
                    if (line != null) {
                        LaserScan laserScan = new LaserScan();
                        laserScan.loadFromString(line);

                        byte []body = Utils.serialize(kryo, laserScan);
                        Map<String, Object> props = new HashMap<String, Object>();
                        props.put("time", System.currentTimeMillis());
                        props.put(TransportConstants.SENSOR_ID, System.currentTimeMillis());
                        Message message = new Message(body, props);
                        try {
                            dataSender.send(message, "test.test.laser_scan");
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                    }
                    Thread.sleep(1000);
                } catch (IOException e) {
                    e.printStackTrace();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private long bestSum;
    private long mapSum;
    private long bestCount;
    private long mapCount;

    private class BestParticleReceiver implements MessageHandler {
        @Override
        public Map<String, String> getProperties() {
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, "test.test.best");
            props.put(MessagingConstants.RABBIT_QUEUE, "test.test.best");
            return props;
        }

        @Override
        public void onMessage(Message message) {
            Object time = message.getProperties().get("time");
            Long t = Long.parseLong(time.toString());
            bestSum += System.currentTimeMillis() - t;
            bestCount++;
            System.out.println("Best Time: " + (System.currentTimeMillis() - t) + "\nAverage Best: " + ((double)(bestSum) / bestCount));
        }
    }

    private class MapReceiver implements MessageHandler {
        @Override
        public Map<String, String> getProperties() {
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, "test.test.map");
            props.put(MessagingConstants.RABBIT_QUEUE, "test.test.map");
            return props;
        }

        @Override
        public void onMessage(Message message) {
            GFSMap map = (GFSMap) Utils.deSerialize(kryo, message.getBody(), GFSMap.class);
            Object time = message.getProperties().get("time");
            Long t = Long.parseLong(time.toString());
            mapCount++;
            mapSum += (System.currentTimeMillis() - t);
            System.out.println("Map Time: " + (System.currentTimeMillis() - t) + "\nAverage Map: " + ((double) (mapSum) / mapCount));
        }
    }
}
