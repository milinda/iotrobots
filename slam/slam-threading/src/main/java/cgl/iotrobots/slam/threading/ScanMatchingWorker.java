package cgl.iotrobots.slam.threading;

import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

public class ScanMatchingWorker implements Runnable {
    private static Logger LOG = LoggerFactory.getLogger(ScanMatchingWorker.class);

    private int startIndex;
    private int endIndex;

    private List<Particle> particles;

    private ScanMatcher matcher;

    private double minimumScore;

    private double[] plainReading;

    public ScanMatchingWorker(List<Particle> particles, int startIndex,
                              int endIndex, ScanMatcher matcher,
                              double[] plainReading, double minimumScore) {
        this.particles = particles;
        this.startIndex = startIndex;
        this.endIndex = endIndex;
        this.matcher = matcher;
        this.plainReading = plainReading;
        this.minimumScore = minimumScore;
    }

    @Override
    public void run() {
        double sumScore = 0;
        for (int i = startIndex; i <= endIndex; i++) {
            Particle it = particles.get(i);
            DoubleOrientedPoint corrected = new DoubleOrientedPoint(0.0, 0.0, 0.0);
            double score = 0, l, s;
            score = matcher.optimize(corrected, it.map, it.pose, plainReading);
            //    it->pose=corrected;
            if (score > minimumScore) {
                it.pose = new DoubleOrientedPoint(corrected);
            } else {
                LOG.info("Scan Matching Failed, using odometry. Likelihood=");
            }

            ScanMatcher.LikeliHoodAndScore score1 = matcher.likelihoodAndScore(it.map, it.pose, plainReading);
            l = score1.l;
            s = score1.s;

            sumScore += score;
            it.weight += l;
            it.weightSum += l;

            //set up the selective copy of the active area
            //by detaching the areas that will be updated
            matcher.invalidateActiveArea();
            matcher.computeActiveArea(it.map, it.pose, plainReading);
        }
    }
}
