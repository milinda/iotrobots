package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.utils.OrientedPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

public class GridSlamProcessor {
    private List<TNode> TNodeVector = new ArrayList<TNode>();
    private BlockingDeque<TNode> TNodeDeque = new LinkedBlockingDeque<TNode>();
    private List<Particle> m_particles = new ArrayList<Particle>();

    ScanMatcher m_matcher;

    MotionModel m_motionModel;

    int m_beams;
    double last_update_time_;
    double period_;

    double minimumScore;

    double resampleThreshold;

    int  m_count, m_readingCount;
    OrientedPoint m_lastPartPose;
    OrientedPoint m_odoPose;
    OrientedPoint m_pose;

    double m_linearDistance, m_angularDistance;

    double neff;

    double xmin;
}
