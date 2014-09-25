package cgl.iotrobots.perf.sensor;

import cgl.iotcloud.core.zk.ClientFactory;
import org.apache.curator.framework.CuratorFramework;
import org.apache.curator.framework.recipes.barriers.DistributedDoubleBarrier;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Test {
    private static Logger LOG = LoggerFactory.getLogger(Test.class);
    private final int testNo;

    private DataGenerator dataGenerator;

    private LatencyWriter writer;

    private List<Long> results = new ArrayList<Long>();

    private int noOfMessages;

    private int dataSize;

    private int freq;

    private String zkServers;

    private CuratorFramework client;

    private int noSensors;

    private DistributedDoubleBarrier barrier;

    private BlockingQueue<byte []> messages = new ArrayBlockingQueue<byte[]>(8);

    private boolean error = false;

    private Lock lock = new ReentrantLock();

    private int sendCount = 0;

    public Test(int noOfMessages, int dataSize, int freq, String zkServers, int noSensors, LatencyWriter writer, int testNo) {
        this.noOfMessages = noOfMessages;
        this.dataSize = dataSize;
        this.freq = freq;
        this.zkServers = zkServers;
        this.writer = writer;
        this.noSensors = noSensors;
        this.testNo = testNo;
    }

    public void start() {
        dataGenerator = new DataGenerator(freq, dataSize, messages, 8);
        ClientFactory clientFactory = ClientFactory.getInstance();
        clientFactory.setAddress(zkServers);
        client = clientFactory.getClient();

        barrier = new DistributedDoubleBarrier(client, "/test/barrier/" + testNo, noSensors);
        try {
            barrier.enter(5, TimeUnit.MINUTES);
        } catch (Exception e) {
            LOG.error("Failed to enter the barrier", e);
        }

        dataGenerator.start();
    }

    public void stop() {
        dataGenerator.stop();

        if (error) {
            LOG.info("Done Receive with test msgRate: {}, msgSize: {}, expectedCount: {}, receivedCount: {}", freq, dataSize, noOfMessages, results.size());
            // immediately write and stop
            int totalWait = 0;
            while (sendCount > results.size() && totalWait < 60000) {
                try {
                    Thread.sleep(1000);
                    totalWait += 1000;
                } catch (InterruptedException ignore) {
                }
            }

            lock.lock();
            try {
                writer.write(getTestName(), results);
            } finally {
                lock.unlock();
            }
        } else {
            // wait until all the results arrive
            int totalWait = 0;
            while (!error && results.size() < noOfMessages && totalWait < 60000) {
                try {
                    LOG.info("Waiting for messages, expected: {}, receive: {}", noOfMessages, results.size());
                    Thread.sleep(1000);
                    totalWait += 1000;
                } catch (InterruptedException ignore) {
                }
            }
            LOG.info("Done Receive with test msgRate: {}, msgSize: {}, expectedCount: {}, receivedCount: {}", freq, dataSize, noOfMessages, results.size());
            lock.lock();
            try {
                writer.write(getTestName(), results);
            } finally {
                lock.unlock();
            }
        }
        try {
            barrier.leave();
        } catch (Exception e) {
            LOG.error("Failed to leave the barrier", e);
        }
    }

    public void incCount() {
        sendCount++;
    }

    public String getTestName() {
        return noSensors + "_" + freq + "_" + dataSize;
    }

    public boolean canContinue() {
        if (results.size() > 10) {
            // check weather last ten is increasing values
            int startIndex = results.size() - 10;
            boolean increasing = true;
            boolean large = true;
            for (int i = startIndex; i < startIndex + 10; i++) {
                if (results.get(i) <= results.get(i -1)) {
                    increasing = false;
                    break;
                }

                if (results.get(i) < 2000) {
                    large = false;
                }
            }

            if (increasing) {
                LOG.error("********** The latencies are increasing, stopping the test **********");
                error = true;
            }

            if (large) {
                LOG.error("********** The latencies are large, stopping the test **********");
                error = true;
            }

            return !increasing;
        }
        return true;
    }

    public void addResult(long result) {
        lock.lock();
        try {
            results.add(result);
        }finally {
            lock.unlock();
        }
    }

    public BlockingQueue<byte[]> getMessages() {
        return messages;
    }
}


