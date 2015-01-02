package cgl.iotrobots.sim;

import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.slam.core.app.GFSAlgorithm;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.gridfastsalm.GridSlamProcessor;
import cgl.iotrobots.slam.streaming.Utils;
import cgl.iotrobots.slam.threading.ParallelGridSlamProcessor;
import cgl.iotrobots.utils.rabbitmq.Message;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FileBasedSimulator {
    public static final int SENSORS = 360;

    public static final double ANGLE = 2 * Math.PI;

    GFSAlgorithm gfsAlgorithm = new GFSAlgorithm();

    BufferedReader br = null;

    public void start() throws InterruptedException {
        // nothing particular in this case
        gfsAlgorithm.gsp_ = new ParallelGridSlamProcessor();
        gfsAlgorithm.init();
        LaserScan scanI = new LaserScan();
        scanI.setAngle_increment(ANGLE / SENSORS);
        scanI.setAngle_max(ANGLE);
        scanI.setAngle_min(0);
        List<Double> ranges  = new ArrayList<Double>();
        for (int i = 0; i < SENSORS; i++) {
            ranges.add(100.0);
        }
        scanI.setRanges(ranges);
        scanI.setRange_min(0);
        scanI.setRangeMax(100);
        scanI.setTimestamp(System.currentTimeMillis());

        gfsAlgorithm.initMapper(scanI);

        try {
            br = new BufferedReader(new FileReader("out.txt"));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        Thread t = new Thread(new SendWorker());
        t.start();

        t.join();
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

                        gfsAlgorithm.laserScan(laserScan);
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

    public static void main(String[] args) throws InterruptedException {
        FileBasedSimulator simulator = new FileBasedSimulator();
        simulator.start();
    }
}
