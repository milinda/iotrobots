package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.sensor.RangeSensor;
import cgl.iotrobots.slam.core.sensor.Sensor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

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
    protected double regScore;
    protected double critScore;
    protected double maxMove;
    protected double linearThresholdDistance;
    protected double angularThresholdDistance;
    protected double obsSigmaGain;

    public AbstractGridSlamProcessor() {
        period_ = 0.0;
        obsSigmaGain = 1;
        resampleThreshold = 0.5;
        minimumScore = 0.;
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
}
