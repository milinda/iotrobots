package cgl.iotrobots.slam.core.scanmatcher;

import cgl.iotrobots.slam.core.utils.OrientedPoint;

public class ScanMatcher {
    double laserMaxRange;
    double usableRange;
    double gaussianSigma;
    double likelihoodSigma;
    int   kernelSize;
    double optAngularDelta;
    double optLinearDelta;
    int optRecursiveIterations;
    int likelihoodSkip;
    double llsamplerange;
    double lasamplerange;
    double llsamplestep;
    double lasamplestep;
    boolean generateMap;
    boolean enlargeStep;
    OrientedPoint laserPose;
}
