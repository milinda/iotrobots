package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.particlefilter.UniformResampler;
import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.sensor.RangeSensor;
import cgl.iotrobots.slam.core.sensor.Sensor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.core.utils.DoublePoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public abstract class AbstractGridSlamProcessor {
    private Logger LOG = LoggerFactory.getLogger(AbstractGridSlamProcessor.class);

    public static final double distanceThresholdCheck = 20;

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
    protected double linearThresholdDistance;
    protected double angularThresholdDistance;
    protected double obsSigmaGain;

    public AbstractGridSlamProcessor() {
        period_ = 100.0;
        obsSigmaGain = 1;
        resampleThreshold = 0.5;
        minimumScore = 0.;
    }

    public abstract void init(int size, double xmin, double ymin, double xmax, double ymax, double delta, DoubleOrientedPoint initialPose);

    public abstract boolean processScan(RangeReading reading, int adaptParticles);

    public abstract void scanMatch(double[] plainReading);

    public abstract void setup();

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

    public int getBestParticleIndex() {
        int bi = 0;
        double bw = -Double.MAX_VALUE;
        for (int i = 0; i < particles.size(); i++)
            if (bw < particles.get(i).weightSum) {
                bw = particles.get(i).weightSum;
                bi = i;
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

    protected double scanMatchParticle(double[] plainReading, double sumScore, Particle it) {
        DoubleOrientedPoint corrected = new DoubleOrientedPoint(0.0, 0.0, 0.0);
        double score = 0, l;
        score = matcher.optimize(corrected, it.map, it.pose, plainReading);
//        it->pose=corrected;
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

        LOG.info("Weigh of the particle: {}", it.weight);

        //set up the selective copy of the active area
        //by detaching the areas that will be updated
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(it.map, it.pose, plainReading);
        return sumScore;
    }

    public void resetTree() {
        // don't calls this function directly, use updateTreeWeights(..) !
        for (Particle it : particles) {
            TNode n = it.node;
            while (n != null) {
                n.accWeight = 0;
                n.visitCounter = 0;
                n = n.parent;
            }
        }
    }

    public double propagateWeight(TNode n, double weight) {
        if (n == null) {
            return weight;
        }
        double w = 0;
        n.visitCounter++;
        n.accWeight += weight;
        if (n.visitCounter == n.childs) {
            w = propagateWeight(n.parent, n.accWeight);
        }
        assert (n.visitCounter <= n.childs);
        return w;
    }

    public double propagateWeights() {
        // don't calls this function directly, use updateTreeWeights(..) !
        // all nodes must be resetted to zero and weights normalized
        // the accumulated weight of the root
        double lastNodeWeight = 0;
        // sum of the weights in the leafs
        double aw = 0;

        int count = 0;
        for (Particle particle : particles) {
            double weight = weights.get(count++);
            aw += weight;
            TNode n = particle.node;
            n.accWeight = weight;
            lastNodeWeight += propagateWeight(n.parent, n.accWeight);
        }

        if (Math.abs(aw - 1.0) > 0.0001 || Math.abs(lastNodeWeight - 1.0) > 0.0001) {
            LOG.error("ERROR: root->accWeight=" + lastNodeWeight + "    sum_leaf_weights=" + aw);
        } else {
            LOG.info("Cooreect weights {} {}", lastNodeWeight, aw);
        }
        return lastNodeWeight;
    }

    public void updateTreeWeights(boolean weightsAlreadyNormalized) {
        if (!weightsAlreadyNormalized) {
            normalize();
        }
        resetTree();
        propagateWeights();
    }

    public void normalize() {
        //normalize the log weights
        double gain = 1. / (obsSigmaGain * particles.size());
        double lmax = -Double.MAX_VALUE;
        for (Particle particle : particles) {
            lmax = particle.weight > lmax ? particle.weight : lmax;
        }

        weights.clear();
        double wcum = 0;
        neff = 0;
        for (Particle particle : particles) {
            double w = Math.exp(gain * (particle.weight - lmax));
//            double w = particle.weight;
            weights.add(w);
            wcum += w;
        }

        neff = 0;
        List<Double> temp = new ArrayList<Double>();
        for (int i = 0; i < weights.size(); i++) {
            double weight = weights.get(i);
            weight = weight / wcum;
            temp.add(weight);
            double w = weight;
            neff += w * w;
        }
        weights.clear();
        weights.addAll(temp);
        neff = 1. / neff;
    }

    public boolean resample(double[] plainReading, int adaptSize, RangeReading reading) {
        boolean hasResampled = false;
        List<TNode> oldGeneration = new ArrayList<TNode>();
        for (Particle m_particle : particles) {
            oldGeneration.add(m_particle.node);
        }

        if (neff < resampleThreshold * particles.size()) {
            LOG.info("neff < resampleThreshold * particles.size() and resampling {} < {}", neff, resampleThreshold * particles.size());
            UniformResampler resampler = new UniformResampler();
            indexes = resampler.resampleIndexes(weights, adaptSize);

            StringBuilder m_outputStream = new StringBuilder("RESAMPLE ").append(indexes.size());
            for (Integer it : indexes) {
                m_outputStream.append(it).append(" ");
            }
            LOG.info(m_outputStream.toString());


            //begin building tree
            List<Particle> temp = new ArrayList<Particle>();
            int j = 0;
            //this is for deleteing the particles which have been resampled away.
            List<Integer> deletedParticles = new ArrayList<Integer>();

            for (int i = 0; i < indexes.size(); i++) {
                while (j < indexes.get(i)) {
                    deletedParticles.add(j);
                    j++;
                }
                if (j == indexes.get(i)) {
                    j++;
                }
                Particle p = new Particle(particles.get(indexes.get(i)));

                TNode node ;
                TNode oldNode = oldGeneration.get(indexes.get(i));
                node = new TNode(p.pose, 0, oldNode, 0);
                node.reading = reading;

                temp.add(p);
                p.node = node;
                p.previousIndex = indexes.get(i);
            }
            while (j < indexes.size()) {
                deletedParticles.add(j);
                j++;
            }

            for (int i = 0; i < deletedParticles.size(); i++) {
                if (deletedParticles.get(i) < particles.size()) {
                    particles.get(deletedParticles.get(i)).node = null;
                }
            }

            particles.clear();
            LOG.debug("Copying Particles and  Registering  scans...");
            for (Particle it : temp) {
                it.setWeight(0);
                matcher.invalidateActiveArea();
                matcher.registerScan(it.map, it.pose, plainReading);
                particles.add(it);
            }
            hasResampled = true;
        } else {
            LOG.info("neff > resampleThreshold * particles.size() and resampling {} > {}", neff, resampleThreshold * particles.size());
            int index = 0;
            LOG.debug("Registering Scans:");
            Iterator<TNode> node_it = oldGeneration.iterator();
            for (Particle it : particles) {
                //create a new node in the particle tree and add it to the old tree
                TNode node = null;
                node = new TNode(it.pose, 0.0, node_it.next(), 0);
//                it.weight = 0;
                node.reading = reading;
                it.node = node;

                matcher.invalidateActiveArea();
                matcher.registerScan(it.map, it.pose, plainReading);
                it.previousIndex = index;
                index++;
            }
        }
        return hasResampled;
    }
}
