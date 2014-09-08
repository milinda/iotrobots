package cgl.iotrobots.perf.sensor;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Random;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.BlockingQueue;

public class DataGenerator {
    private static Logger LOG = LoggerFactory.getLogger(DataGenerator.class);

    private int freq;

    private int size;

    private BlockingQueue<byte []> messages;

    private Timer timer;

    private byte[]data;

    private int capacity = 64;

    public DataGenerator(int freq, int size, BlockingQueue<byte[]> messages, int capacity) {
        this.freq = freq;
        this.size = size;
        this.messages = messages;
        this.capacity = capacity;
    }

    public void start() {
        data = new byte[size];
        new Random().nextBytes(data);

        TimerTask task = new Task();
        timer = new Timer(true);
        timer.scheduleAtFixedRate(task, 0, 1000 / freq);
    }

    public void stop() {
        timer.cancel();
    }

    private class Task extends TimerTask {
        @Override
        public void run() {
            try {
                if (messages.size() == capacity) {
                    LOG.warn("We are blocking and cannot handle this frequency");
                }
                messages.put(data);
            } catch (InterruptedException e) {
                LOG.error("Failed the put", e);
            }
        }
    }

    public int getFreq() {
        return freq;
    }

    public int getSize() {
        return size;
    }

    public BlockingQueue<byte[]> getMessages() {
        return messages;
    }
}
