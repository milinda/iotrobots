package cgl.iotrobots.perf.sensor;

import org.apache.curator.framework.CuratorFramework;
import org.apache.curator.framework.CuratorFrameworkFactory;
import org.apache.curator.framework.recipes.barriers.DistributedDoubleBarrier;
import org.apache.curator.retry.ExponentialBackoffRetry;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;

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
        client = CuratorFrameworkFactory.newClient(zkServers, new ExponentialBackoffRetry(1000, 3));
        client.start();

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
            writer.write(getTestName(), results);
        } else {
            // wait until all the results arrive
            while (!error && results.size() < noOfMessages) {
                try {
                    LOG.info("Waiting for messages, expected: {}, receive: {}", noOfMessages, results.size());
                    Thread.sleep(10);
                } catch (InterruptedException ignore) {
                }
            }
            LOG.info("Done Receive with test msgRate: {}, msgSize: {}, expectedCount: {}, receivedCount: {}", freq, dataSize, noOfMessages, results.size());
            writer.write(getTestName(), results);
        }
        try {
            barrier.leave();
        } catch (Exception e) {
            LOG.error("Failed to leave the barrier", e);
        }

        if (client != null) {
            client.close();
        }
    }

    public String getTestName() {
        return noSensors + "_" + freq + "_" + dataSize;
    }

    public boolean canContinue() {
        if (results.size() > 10) {
            // check weather last ten is increasing values
            int startIndex = results.size() - 10;
            boolean increasing = true;
            for (int i = startIndex; i < startIndex + 10; i++) {
                if (results.get(i) <= results.get(i -1)) {
                    increasing = false;
                    break;
                }
            }

            if (increasing) {
                LOG.error("********** The latencies are increasing, stopping the test **********");
                error = true;
            }

            return !increasing;
        }
        return true;
    }

    public void addResult(long result) {
        results.add(result);
    }

    public BlockingQueue<byte[]> getMessages() {
        return messages;
    }
}


