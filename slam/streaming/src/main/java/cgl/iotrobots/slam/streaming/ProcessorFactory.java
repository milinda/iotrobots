package cgl.iotrobots.slam.streaming;

import cgl.iotrobots.slam.core.GFSConfiguration;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;

/**
 * Creates objects of the processors using the Configurations
 */
public class ProcessorFactory {
    public static DistributedScanMatcher createMatcher(GFSConfiguration cfg) {
        DistributedScanMatcher scanMatcher = new DistributedScanMatcher();

        scanMatcher.setMatchingParameters(cfg.getMaxURage(), cfg.getMaxRange(), cfg.getGaussianSigma(),
                cfg.getKernelSize(), cfg.getLinearThresholdDistance(),
                cfg.getAngularThresholdDistance(), cfg.getOptRecursiveIterations(),
                cfg.getLikelihoodSigma(), cfg.getLikelihoodGain(), cfg.getLikelihoodSkip());
        scanMatcher.setMotionModelParameters(cfg.getSrr(), cfg.getSrt(), cfg.getStr(), cfg.getStt());
        scanMatcher.setUpdatePeriod_(cfg.getUpdatePeriod());
        scanMatcher.setMinimumScore(cfg.getMinimumScore());
        scanMatcher.init(cfg.getNoOfParticles(), cfg.getXmin(), cfg.getYmin(),
                cfg.getXmax(), cfg.getYmax(), cfg.getDelta(), new DoubleOrientedPoint(0.0, 0.0, 0.0));

        return scanMatcher;
    }

    public static DistributedReSampler createReSampler(GFSConfiguration cfg) {
        DistributedReSampler reSampler = new DistributedReSampler();

        reSampler.setMatchingParameters(cfg.getMaxURage(), cfg.getMaxRange(), cfg.getGaussianSigma(),
                cfg.getKernelSize(), cfg.getLinearThresholdDistance(),
                cfg.getAngularThresholdDistance(), cfg.getOptRecursiveIterations(),
                cfg.getLikelihoodSigma(), cfg.getLikelihoodGain(), cfg.getLikelihoodSkip());
        reSampler.setMotionModelParameters(cfg.getSrr(), cfg.getSrt(), cfg.getStr(), cfg.getStt());
        reSampler.setUpdatePeriod_(cfg.getUpdatePeriod());
        reSampler.setMinimumScore(cfg.getMinimumScore());
        reSampler.init(cfg.getNoOfParticles(), cfg.getXmin(), cfg.getYmin(),
                cfg.getXmax(), cfg.getYmax(), cfg.getDelta(), new DoubleOrientedPoint(0.0, 0.0, 0.0));

        return reSampler;
    }
}
