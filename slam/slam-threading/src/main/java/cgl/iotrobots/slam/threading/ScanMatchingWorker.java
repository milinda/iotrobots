package cgl.iotrobots.slam.threading;

import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;
import java.util.concurrent.Semaphore;

public class ScanMatchingWorker implements Runnable {
    private static Logger LOG = LoggerFactory.getLogger(ScanMatchingWorker.class);

    private int startIndex;
    private int endIndex;

    private List<Particle> particles;

    private ScanMatcher matcher;

    private double minimumScore;

    private double[] plainReading;

    private Semaphore semaphore;

    public ScanMatchingWorker(List<Particle> particles, int startIndex,
                              int endIndex, ScanMatcher matcher,
                              double[] plainReading, double minimumScore, Semaphore semaphore) {
        this.particles = particles;
        this.startIndex = startIndex;
        this.endIndex = endIndex;
        this.matcher = matcher;
        this.plainReading = plainReading;
        this.minimumScore = minimumScore;
        this.semaphore = semaphore;
    }

    @Override
    public void run() {
        double sumScore = 0;
        for (int i = startIndex; i <= endIndex; i++) {
            Particle it = particles.get(i);
            DoubleOrientedPoint corrected = new DoubleOrientedPoint(0.0, 0.0, 0.0);
            double score = 0, l, s;
            long time = System.currentTimeMillis();
            score = matcher.optimize(corrected, it.map, it.pose, plainReading);
            // System.out.println("Scan match time: " + (System.currentTimeMillis() - time));
            if (score > minimumScore) {
                LOG.info("Score {}: Correcting the position from {} to {}", score, it.pose, corrected);
                it.pose = new DoubleOrientedPoint(corrected);
            } else {
                LOG.info("Score {}:, Scan Matching Failed, using odometry", score);
            }

            ScanMatcher.LikeliHoodAndScore score1 = matcher.likelihoodAndScore(it.map, it.pose, plainReading);
            l = score1.l;
            s = score1.s;

            sumScore += score;
            it.weight += l;
            it.weightSum += l;
            LOG.info("Weigh of the particle: {}", it.weight);

            //set up the selective copy of the active area
            //by detaching the areas that will be updated
            matcher.invalidateActiveArea();
            matcher.computeActiveArea(it.map, it.pose, plainReading);

            semaphore.release();
        }
    }
}
