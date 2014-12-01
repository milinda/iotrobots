package cgl.iotrobots.slam.threading;

import cgl.iotrobots.slam.core.gridfastsalm.MotionModel;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;

import java.util.ArrayList;
import java.util.List;

public class ParallelGridSlamProcessor {
    private static final double m_distanceThresholdCheck = 20;

    private List<Particle> m_particles = new ArrayList<Particle>();

    List<Integer> m_indexes = new ArrayList<Integer>();
    List<Double> m_weights = new ArrayList<Double>();

    ScanMatcher m_matcher = new ScanMatcher();
    MotionModel m_motionModel = new MotionModel();

    int m_beams;
    double last_update_time_;
    double period_;

    double m_minimumScore;

    double m_resampleThreshold;

    int m_count, m_readingCount;
    DoubleOrientedPoint m_lastPartPose = new DoubleOrientedPoint(0.0, 0.0, 0.0);
    DoubleOrientedPoint m_odoPose = new DoubleOrientedPoint(0.0, 0.0, 0.0);
    DoubleOrientedPoint m_pose = new DoubleOrientedPoint(0.0, 0.0, 0.0);

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


}
