package cgl.iotrobots.sim;

import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.slam.core.app.GFSMap;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.streaming.Utils;
import cgl.iotrobots.slam.utils.FileIO;
import cgl.iotrobots.utils.rabbitmq.*;
import com.esotericsoftware.kryo.Kryo;

import java.util.HashMap;
import java.util.Map;

public class FileBasedDistributedSimulator {
    RabbitMQSender dataSender;
    RabbitMQSender controlSender;
    RabbitMQReceiver receiver;
    RabbitMQReceiver bestReceiver;
    FileIO fileIO;
    FileIO resultMapIO;
    FileIO resultBestIO;
    Kryo kryo = new Kryo();

    public FileBasedDistributedSimulator(String url, String file, String test) {
        try {
            dataSender = new RabbitMQSender(url, "simbard_laser");
            controlSender = new RabbitMQSender(url, "simbard_control");
            receiver = new RabbitMQReceiver(url, "simbard_map");
            bestReceiver = new RabbitMQReceiver(url, "simbard_best");

            dataSender.open();
            controlSender.open();
            receiver.listen(new MapReceiver());
            bestReceiver.listen(new BestParticleReceiver());

            fileIO = new FileIO(file, false);
            resultMapIO = new FileIO(test + "_map", true);
            resultBestIO = new FileIO(test + "_best", true);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void start() throws InterruptedException {
        byte[] body = "start".getBytes();
        Map<String, Object> props = new HashMap<String, Object>();
        props.put("time", System.currentTimeMillis());
        props.put(TransportConstants.SENSOR_ID, "hellllo");
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
        if (args.length < 3) {
            System.out.println("Please specify amqp url, filename and test name as arguments");
        }

        FileBasedDistributedSimulator fileBasedSimulator = new FileBasedDistributedSimulator(args[0], args[1], args[2]);
        fileBasedSimulator.start();
    }

    private class SendWorker implements Runnable {
        @Override
        public void run() {
            while (true) {
                try {
                    LaserScan scan = fileIO.read();
                    if (scan != null) {
                        byte []body = Utils.serialize(kryo, scan);
                        Map<String, Object> props = new HashMap<String, Object>();
                        props.put("time", System.currentTimeMillis());
                        props.put(TransportConstants.SENSOR_ID, "hellllo");
                        Message message = new Message(body, props);
                        try {
                            dataSender.send(message, "test.test.laser_scan");
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                    } else {
                        break;
                    }
                    Thread.sleep(1000);
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
            resultBestIO.writeResult((System.currentTimeMillis() - t) + "");
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
            resultMapIO.writeResult((System.currentTimeMillis() - t) + "");
            System.out.println("Map Time: " + (System.currentTimeMillis() - t) + "\nAverage Map: " + ((double) (mapSum) / mapCount));
        }
    }
}
