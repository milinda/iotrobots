package cgl.iotrobots.sim;

import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.slam.core.app.GFSMap;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.streaming.Utils;
import cgl.iotrobots.slam.streaming.msgs.Trace;
import cgl.iotrobots.slam.utils.FileIO;
import cgl.iotrobots.utils.rabbitmq.*;
import com.esotericsoftware.kryo.Kryo;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class FileBasedDistributedSimulator {
    RabbitMQSender dataSender;
    RabbitMQSender controlSender;
    RabbitMQReceiver receiver;
    RabbitMQReceiver bestReceiver;
    FileIO fileIO;
    FileIO resultMapIO;
    FileIO resultBestIO;
    SlamDataReader dataReader;
    Kryo kryo = new Kryo();
    boolean simbard;
    MapUI mapUI;
    long sleepTime = 1000;

//    Lock lock;
//    Condition sendWait;
//    Condition receiveWait;

    public FileBasedDistributedSimulator(String url, String file, String test, boolean simbard, boolean ui, long sleepTime) {
        try {
//            this.lock = new ReentrantLock();
//            this.sendWait = lock.newCondition();
//            this.receiveWait = lock.newCondition();
            dataSender = new RabbitMQSender(url, "simbard_laser");
            controlSender = new RabbitMQSender(url, "simbard_control");
            receiver = new RabbitMQReceiver(url, "simbard_map");
            bestReceiver = new RabbitMQReceiver(url, "simbard_best");

            // Utils.registerClasses(kryo);

            dataSender.open();
            controlSender.open();
            receiver.listen(new MapReceiver());
            bestReceiver.listen(new BestParticleReceiver());

            if (simbard) {
                fileIO = new FileIO(file, false);
            } else {
                dataReader = new SlamDataReader(file);
            }
            resultMapIO = new FileIO(test + "_map", true);
            resultBestIO = new FileIO(test + "_best", true);
            if (ui) {
                mapUI = new MapUI();
            }
            this.simbard = simbard;
            this.sleepTime = sleepTime;
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

        FileBasedDistributedSimulator fileBasedSimulator = new FileBasedDistributedSimulator(args[0], args[1], args[2], Boolean.parseBoolean(args[3]), Boolean.parseBoolean(args[4]), Long.parseLong(args[5]));
        fileBasedSimulator.start();
    }

    private boolean send = false;

    private class SendWorker implements Runnable {
        @Override
        public void run() {
            long t0 = System.currentTimeMillis();
            while (true) {
//                lock.lock();
//                try {
//                    while (send) {
//                        System.out.println("send wait");
//                        receiveWait.await(1000, TimeUnit.MILLISECONDS);
//                        break;
//                    }
                    LaserScan scan;
                    t0 = System.currentTimeMillis();
                    if (simbard) {
                        scan = fileIO.read();
                    } else {
                        scan = dataReader.read();
                    }
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
                        try {
                            Thread.sleep(sleepTime);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    } else {
                        System.out.println("We are done!!");
                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        System.exit(0);
                        return;
                    }

                    send = true;
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                } finally {
//                    lock.unlock();
//                }
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
//            lock.lock();
//            try {
              //  System.out.println("receive");
                Object time = message.getProperties().get("time");
                Long t = Long.parseLong(time.toString());
                bestSum += System.currentTimeMillis() - t;
            long receiveTime = System.currentTimeMillis();
                bestCount++;
            Trace trace = (Trace) Utils.deSerialize(kryo, message.getBody(), Trace.class);
                resultBestIO.writeResult((receiveTime - t) + " ," + trace.serialize());
//                System.out.println("Best Time: " + (System.currentTimeMillis() - t) + "\nAverage Best: " + ((double) (bestSum) / bestCount));
            //System.out.println((System.currentTimeMillis() - t) + " ," + trace.serialize());
                // send = false;
                // receiveWait.signal();
//            }finally {
//                lock.unlock();
//            }
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
            if (mapUI != null) {
                mapUI.setMap(map);
            }
        }
    }
}
