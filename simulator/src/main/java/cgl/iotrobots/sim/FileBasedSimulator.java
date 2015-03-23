package cgl.iotrobots.sim;

import cgl.iotrobots.slam.core.app.GFSAlgorithm;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.gridfastsalm.GridSlamProcessor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.threading.ParallelGridSlamProcessor;
import cgl.iotrobots.slam.utils.FileIO;
import cgl.iotrobots.slam.utils.RosMapPublisher;
import cgl.iotrobots.slam.utils.RosTurtle;
import cgl.iotrobots.slam.utils.TurtleUtils;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class FileBasedSimulator {
    public static final int SENSORS = 360;

    public static final double ANGLE = 2 * Math.PI;

    GFSAlgorithm gfsAlgorithm = new GFSAlgorithm();

    int parallel = 2;

    FileIO fileIO;

    SlamDataReader dataReader;

    MapUI mapUI;

    boolean simbard = true;

    RosMapPublisher mapPublisher = new RosMapPublisher();

    boolean ui = false;

    public void start(boolean parallel, String file, int particles, boolean simbard, boolean ui) throws InterruptedException {
        // nothing particular in this case
        if (!parallel) {
            gfsAlgorithm.gsp = new GridSlamProcessor();
        } else {
            gfsAlgorithm.gsp = new ParallelGridSlamProcessor();
        }
        this.ui = ui;
        if (ui) {
            mapUI = new MapUI();
        }
        gfsAlgorithm.init();
        gfsAlgorithm.setParticles(particles);
        LaserScan scanI = new LaserScan();
        scanI.setAngleIncrement(ANGLE / SENSORS);
        scanI.setAngleMax(ANGLE);
        scanI.setAngleMin(0);
        List<Double> ranges  = new ArrayList<Double>();
        for (int i = 0; i < SENSORS; i++) {
            ranges.add(100.0);
        }
        scanI.setRanges(ranges);
        scanI.setRangeMin(0);
        scanI.setRangeMax(100);
        scanI.setTimestamp(System.currentTimeMillis());
        scanI.setPose(new DoubleOrientedPoint(0.0, 0.0, 0.0));

        gfsAlgorithm.initMapper(scanI);

//        RosTurtle rosTurtle = new RosTurtle();
//        TurtleUtils.connectToRos(rosTurtle);
        if (ui) {
            TurtleUtils.connectToRos(mapPublisher);
        }

        if (simbard) {
            fileIO = new FileIO(file, false);
        } else {
            dataReader = new SlamDataReader(file);
        }

        this.simbard = simbard;

        Thread t = new Thread(new SendWorker());
        t.start();

        t.join();
    }

    private class SendWorker implements Runnable {
        @Override
        public void run() {
            LaserScan oldScan = null;
            while (true) {
                LaserScan scan;
                if (simbard) {
                    scan = fileIO.read();
                } else {
                    scan = dataReader.read();
                }
                if (scan != null) {
                    gfsAlgorithm.laserScan(scan);
                    if (ui) {
                        mapUI.setMap(gfsAlgorithm.getMap());
                        mapPublisher.addMap(gfsAlgorithm.getMap());
                    }
                } else {
                    gfsAlgorithm.setLastMapUpdate(0);
                    if (oldScan != null) {
                        gfsAlgorithm.laserScan(oldScan);
                        if (ui) {
                            mapUI.setMap(gfsAlgorithm.getMap());
                            mapPublisher.addMap(gfsAlgorithm.getMap());
                        }
                        System.out.println("We are done!!");
                        return;
                    }
                }
                oldScan = scan;
            }
        }
    }

    public static void main(String[] args) throws InterruptedException {
        FileBasedSimulator simulator = new FileBasedSimulator();
        if (args.length > 2) {
            simulator.start(Boolean.parseBoolean(args[0]), args[1], Integer.parseInt(args[2]), Boolean.parseBoolean(args[3]), Boolean.parseBoolean(args[5]));
            simulator.parallel = Integer.parseInt(args[4]);
        } else if (args.length == 1) {
            simulator.start(false, args[0], 20, true, false);
        }
    }
}
