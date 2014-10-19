package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.sensor.OdometryReading;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.utils.OrientedPoint;
import cgl.iotrobots.slam.core.utils.Point;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

public class GridSlamProcessor {
    private List<TNode> TNodeVector = new ArrayList<TNode>();
    private BlockingDeque<TNode> TNodeDeque = new LinkedBlockingDeque<TNode>();
    private List<Particle> m_particles = new ArrayList<Particle>();

    private

    ScanMatcher m_matcher;

    MotionModel m_motionModel;

    int m_beams;
    double last_update_time_;
    double period_;

    double m_minimumScore;

    double m_resampleThreshold;

    int  m_count, m_readingCount;
    OrientedPoint m_lastPartPose;
    OrientedPoint m_odoPose;
    OrientedPoint m_pose;

    double m_linearDistance, m_angularDistance;

    double m_neff;

    double m_xmin;
    double m_ymin;
    double m_xmax;
    double m_ymax;

    double m_delta;
    double m_regScore;
    double m_critScore;
    double m_maxMove;
    double m_linearThresholdDistance;
    double m_angularThresholdDistance;
    double m_obsSigmaGain;

    public GridSlamProcessor() {
        period_ = 5.0;
        m_obsSigmaGain=1;
        m_resampleThreshold=0.5;
        m_minimumScore=0.;
    }

    void setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt,
                               int iterations, double likelihoodSigma, double likelihoodGain, int likelihoodSkip) {
        m_obsSigmaGain = likelihoodGain;
        m_matcher.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma, likelihoodSkip);
    }

    public void setMotionModelParameters
            (double srr, double srt, double str, double stt) {
        m_motionModel.srr = srr;
        m_motionModel.srt = srt;
        m_motionModel.str = str;
        m_motionModel.stt = stt;
    }

    public void setUpdateDistances(double linear, double angular, double resampleThreshold) {
        m_linearThresholdDistance = linear;
        m_angularThresholdDistance = angular;
        m_resampleThreshold = resampleThreshold;
    }

    public void init(int size, double xmin, double ymin, double xmax, double ymax, double delta, OrientedPoint initialPose){
        m_xmin = xmin;
        m_ymin = ymin;
        m_xmax = xmax;
        m_ymax = ymax;
        m_delta = delta;

        m_particles.clear();

        TNode node = new TNode(initialPose, 0, null, 0);
        GMap lmap = new GMap(new Point<Double>((xmin + xmax) * .5, (ymin + ymax) * .5), xmax-xmin, ymax-ymin, delta);
        for (int i=0; i<size; i++){
            int lastIndex = m_particles.size() - 1;
            m_particles.add(new Particle(lmap));
            m_particles.get(lastIndex).pose=initialPose;
            m_particles.get(lastIndex).previousPose=initialPose;
            m_particles.get(lastIndex).setWeight(0);
            m_particles.get(lastIndex).previousIndex=0;
            // this is not needed
            //		m_particles.back().node=new TNode(initialPose, 0, node, 0);
            // we use the root directly
            m_particles.get(lastIndex).node= node;
        }


        m_neff=(double)size;
        m_count=0;
        m_readingCount=0;
        m_linearDistance=m_angularDistance=0;
    }

    public void processTruePos(OdometryReading o){

    }

    public void processScan(RangeReading reading, int adaptParticles) {

    }

}
