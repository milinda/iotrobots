package cgl.iotrobots.slam.streaming;

import cgl.iotrobots.slam.core.grid.IGMap;
import cgl.iotrobots.slam.core.grid.MapFactory;
import cgl.iotrobots.slam.core.gridfastsalm.MotionModel;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.gridfastsalm.TNode;
import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.sensor.RangeSensor;
import cgl.iotrobots.slam.core.sensor.Sensor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.core.utils.DoublePoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class DistributedScanMatcher {
    private static Logger LOG = LoggerFactory.getLogger(DistributedScanMatcher.class);

    public static final double distanceThresholdCheck = 1;

    protected List<Particle> particles = new ArrayList<Particle>();

    protected List<Integer> indexes = new ArrayList<Integer>();
    protected List<Double> weights = new ArrayList<Double>();

    protected ScanMatcher matcher = new ScanMatcher();
    protected MotionModel motionModel = new MotionModel();

    protected int beams;
    protected double last_update_time_;
    protected double period_;

    protected double minimumScore;

    protected double resampleThreshold;

    protected int count, readingCount;
    protected DoubleOrientedPoint lastPartPose = new DoubleOrientedPoint(0.0, 0.0, 0.0);
    protected DoubleOrientedPoint odoPose = new DoubleOrientedPoint(0.0, 0.0, 0.0);
    protected DoubleOrientedPoint pose = new DoubleOrientedPoint(0.0, 0.0, 0.0);

    protected double linearDistance, angularDistance;

    protected double neff;

    protected double xmin;
    protected double ymin;
    protected double xmax;
    protected double ymax;

    protected double delta;
    protected double regScore;
    protected double critScore;
    protected double maxMove;
    protected double linearThresholdDistance;
    protected double angularThresholdDistance;
    protected double obsSigmaGain;

    private List<Integer> activeParticles = new ArrayList<Integer>();

    // this lock must not be here. For now we will use it
    private Lock lock = new ReentrantLock();

    public DistributedScanMatcher() {
        period_ = 0.0;
        obsSigmaGain = 1;
        resampleThreshold = 0.5;
        minimumScore = 0.;
    }

    public List<Integer> getActiveParticles() {
        return activeParticles;
    }

    public void setSrr(double srr) {
        motionModel.srr = srr;
    }

    public void setSrt(double srt) {
        motionModel.srt = srt;
    }

    public MotionModel getMotionModel() {
        return motionModel;
    }

    public ScanMatcher getMatcher() {
        return matcher;
    }

    public List<Particle> getParticles() {
        return particles;
    }

    public void clearActiveParticles() {
        lock.lock();
        try {
            activeParticles.clear();
        } finally {
            lock.unlock();
        }
    }

    public void addActiveParticle(int index) {
        lock.lock();
        try {
            if (!activeParticles.contains(index)) {
                activeParticles.add(index);
            }
        } finally {
            lock.unlock();
        }
    }

    public int getBestParticleIndex() {
        int bi = 0;
        double bw = -Double.MAX_VALUE;
        for (int i = 0; i < activeParticles.size(); i++) {
            int particleIndex = activeParticles.get(i);
            if (bw < particles.get(particleIndex).weightSum) {
                bw = particles.get(particleIndex).weightSum;
                bi = particleIndex;
            }
        }
        return bi;
    }

    public void setMinimumScore(double m_minimumScore) {
        this.minimumScore = m_minimumScore;
    }

    public void setUpdatePeriod_(double period_) {
        this.period_ = period_;
    }

    public void setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt,
                                      int iterations, double likelihoodSigma, double likelihoodGain, int likelihoodSkip) {
        obsSigmaGain = likelihoodGain;
        matcher.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma, likelihoodSkip);
        LOG.info(" -maxUrange " + urange
                + " -maxUrange " + range
                + " -sigma     " + sigma
                + " -kernelSize " + kernsize
                + " -lstep " + lopt
                + " -lobsGain " + obsSigmaGain
                + " -astep " + aopt);
    }

    public void setMotionModelParameters
            (double srr, double srt, double str, double stt) {
        motionModel.srr = srr;
        motionModel.srt = srt;
        motionModel.str = str;
        motionModel.stt = stt;
        LOG.info(" -srr " + srr + " -srt " + srt + " -str " + str + " -stt " + stt);
    }

    public void setUpdateDistances(double linear, double angular, double resampleThreshold) {
        linearThresholdDistance = linear;
        angularThresholdDistance = angular;
        this.resampleThreshold = resampleThreshold;
        LOG.info(" -linearUpdate " + linear
                + " -angularUpdate " + angular
                + " -resampleThreshold " + this.resampleThreshold);
    }

    public void setActiveParticles(List<Integer> activeParticles) {
        this.activeParticles.clear();
        this.activeParticles.addAll(activeParticles);
    }

    public void setSensorMap(Map<String, Sensor> smap) {
        /*
          Construct the angle table for the sensor
          FIXME For now detect the readings of only the front laser, and assume its pose is in the center of the robot
        */
        RangeSensor rangeSensor = (RangeSensor) smap.get("ROBOTLASER1");
        beams = rangeSensor.beams().size();
        double[] angles = new double[rangeSensor.beams().size()];
        for (int i = 0; i < beams; i++) {
            angles[i] = rangeSensor.beams().get(i).pose.theta;
        }
        matcher.setLaserParameters(beams, angles, rangeSensor.getPose());
    }



    public void init(int size, double xmin, double ymin, double xmax, double ymax, double delta, DoubleOrientedPoint initialPose) {
        this.xmin = xmin;
        this.ymin = ymin;
        this.xmax = xmax;
        this.ymax = ymax;
        this.delta = delta;

        LOG.info(" -xmin " + this.xmin + " -xmax " + this.xmax + " -ymin " + this.ymin
                + " -ymax " + this.ymax + " -delta " + this.delta + " -particles " + size);

        particles.clear();

        for (int i = 0; i < size; i++) {
            IGMap lmap = MapFactory.create(new DoublePoint((xmin + xmax) * .5, (ymin + ymax) * .5), xmax - xmin, ymax - ymin, delta);
            Particle p = new Particle(lmap);

            p.pose = new DoubleOrientedPoint(initialPose);
            p.previousPose = initialPose;
            p.setWeight(0);
            p.previousIndex = 0;
            particles.add(p);
            // we use the root directly
            p.node = new TNode(initialPose, 0, null, 0);
        }

        neff = (double) size;
        count = 0;
        readingCount = 0;
        linearDistance = angularDistance = 0;
    }

    public boolean processScan(RangeReading reading, int adaptParticles) {
        long t0 = System.currentTimeMillis();
        DoubleOrientedPoint relPose = reading.getPose();
        if (count == 0) {
            lastPartPose = odoPose = relPose;
        }

        for (Particle p : particles) {
            p.pose = motionModel.drawFromMotion(p.pose, relPose, odoPose);
//            LOG.info("Using position: {}", p.pose);
        }

        DoubleOrientedPoint move = DoubleOrientedPoint.minus(relPose, odoPose);
        move.theta = Math.atan2(Math.sin(move.theta), Math.cos(move.theta));
        linearDistance += Math.sqrt(DoubleOrientedPoint.mulN(move, move));
        angularDistance += Math.abs(move.theta);

        // if the robot jumps throw a warning
        if (linearDistance > distanceThresholdCheck) {
            LOG.error("The robot jumped too much *********************************************************** ");
        }

        odoPose = relPose;
        boolean processed = false;

        // process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
        if (count == 0
                || linearDistance >= linearThresholdDistance
                || angularDistance >= angularThresholdDistance
                || (period_ >= 0.0 && (reading.getTime() - last_update_time_) > period_)) {
            last_update_time_ = reading.getTime();

            //this is for converting the reading in a scan-matcher feedable form
            if (reading.size() != beams) {
                throw new IllegalStateException("reading should contain " + beams + " beams");
            }
            double[] plainReading = new double[beams];
            for (int i = 0; i < beams; i++) {
                plainReading[i] = reading.get(i);
            }

            RangeReading readingCopy =
                    new RangeReading(reading.size(), reading.toArray(new Double[reading.size()]),
                            (RangeSensor) reading.getSensor(),
                            reading.getTime());

            if (count > 0) {
                scanMatch(plainReading);
                // updateTreeWeights(false);
                // resample(plainReading, adaptParticles, readingCopy);
            } else {
                for (int index : activeParticles) {
                    Particle it = particles.get(index);
                    matcher.invalidateActiveArea();
                    matcher.computeActiveArea(it.map, it.pose, plainReading);
                    matcher.registerScan(it.map, it.pose, plainReading);

                    // not needed anymore, particles refer to the root in the beginning!
                    TNode node = new TNode(it.pose, 0., it.node, 0);
                    node.reading = readingCopy;
                    it.node = node;
                }
            }

            lastPartPose = odoPose; //update the past pose for the next iteration
            linearDistance = 0;
            angularDistance = 0;
            count++;
            processed = true;

            //keep ready for the next step
            for (Particle it : particles) {
                it.previousPose = it.pose;
            }

        }
        readingCount++;
        LOG.info("Time in prcess scan: {} *****************************", (System.currentTimeMillis() - t0));
        return processed;
    }

    protected double scanMatchParticle(double[] plainReading, double sumScore, Particle it) {
        DoubleOrientedPoint corrected = new DoubleOrientedPoint(0.0, 0.0, 0.0);
        double score, l;
        score = matcher.optimize(corrected, it.map, it.pose, plainReading);
        //    it->pose=corrected;
        if (score > minimumScore) {
            LOG.info("Score {}: Correcting the position from {} to {}", score, it.pose, corrected);
            it.pose = new DoubleOrientedPoint(corrected);
        } else {
            LOG.info("Score {}:, Scan Matching Failed, using odometry. Likelihood: last pose P{}, odom pose {}", score, lastPartPose, odoPose);
        }

        ScanMatcher.LikeliHoodAndScore score1 = matcher.likelihoodAndScore(it.map, it.pose, plainReading);
        l = score1.l;

        sumScore += score;
        it.weight += l;
        it.weightSum += l;

        //set up the selective copy of the active area
        //by detaching the areas that will be updated
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(it.map, it.pose, plainReading);
        return sumScore;
    }

    public void processAfterReSampling(double []plainReading) {
        lock.lock();
        try {
            for (int i : activeParticles) {
                Particle it = particles.get(i);
                matcher.invalidateActiveArea();
                matcher.registerScan(it.map, it.pose, plainReading);
                // particles.add(it);
            }
        } finally {
            lock.unlock();
        }
    }

    public void postProcessingWithoutReSampling(double []plainReading, RangeReading reading) {
        List<TNode> oldGeneration = new ArrayList<TNode>();
        for (int i : activeParticles) {
            Particle m_particle = particles.get(i);
            oldGeneration.add(m_particle.node);
        }
        int index = 0;
        LOG.debug("Registering Scans:");
        Iterator<TNode> node_it = oldGeneration.iterator();
        lock.lock();
        try {
            for (int i : activeParticles) {
                Particle it = particles.get(i);
                //create a new node in the particle tree and add it to the old tree
                TNode node = null;
                node = new TNode(it.pose, 0.0, node_it.next(), 0);

                node.reading = reading;
                it.node = node;

                matcher.invalidateActiveArea();
                matcher.registerScan(it.map, it.pose, plainReading);
                it.previousIndex = index;
                index++;
            }
        } finally {
            lock.unlock();
        }
    }

    /**
     * Just scan match every single particle.
     * If the scan matching fails, the particle gets a default likelihood.
     */
    public void scanMatch(double[] plainReading) {
        // sample a new pose from each scan in the reference
        double sumScore = 0;
        lock.lock();
        try {
            for (int index : activeParticles) {
                Particle it = particles.get(index);
                sumScore += scanMatchParticle(plainReading, sumScore, it);
            }
        }finally {
            lock.unlock();
        }
        LOG.info("Average Scan Matching Score =" + sumScore / particles.size());
    }
}
