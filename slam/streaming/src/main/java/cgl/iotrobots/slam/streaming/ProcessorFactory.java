package cgl.iotrobots.slam.streaming;

import cgl.iotrobots.slam.core.GFSConfiguration;
import cgl.iotrobots.slam.core.app.MapUpdater;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import scala.Int;

import java.util.ArrayList;
import java.util.List;

/**
 * Creates objects of the processors using the Configurations
 */
public class ProcessorFactory {
    public static DScanMatcher createScanMatcher(GFSConfiguration cfg, List<Integer> activeParticles) {
        DScanMatcher scanMatcher = new DScanMatcher();

        scanMatcher.setMatchingParameters(cfg.getMaxURage(), cfg.getMaxRange(), cfg.getGaussianSigma(),
                cfg.getKernelSize(), cfg.getLinearThresholdDistance(),
                cfg.getAngularThresholdDistance(), cfg.getOptRecursiveIterations(),
                cfg.getLikelihoodSigma(), cfg.getLikelihoodGain(), cfg.getLikelihoodSkip());
        scanMatcher.setMotionModelParameters(cfg.getSrr(), cfg.getSrt(), cfg.getStr(), cfg.getStt());
        scanMatcher.setUpdatePeriod(cfg.getUpdatePeriod());
        scanMatcher.setMinimumScore(cfg.getMinimumScore());
        scanMatcher.init(cfg.getNoOfParticles(), cfg.getXmin(), cfg.getYmin(),
                cfg.getXmax(), cfg.getYmax(), cfg.getDelta(), new DoubleOrientedPoint(0.0, 0.0, 0.0), activeParticles);
        scanMatcher.getActiveParticles().addAll(activeParticles);
        return scanMatcher;
    }

    public static DistributedReSampler createReSampler(GFSConfiguration cfg) {
        DistributedReSampler reSampler = new DistributedReSampler(cfg.getObsSigmaGain(), cfg.getResampleThreshold(), cfg.getNoOfParticles());
        return reSampler;
    }

    public static MapUpdater createMapBuilder(GFSConfiguration cfg) {
        return new MapUpdater(cfg.getMaxRange(), cfg.getMaxURage(), cfg.getXmin(), cfg.getYmin(), cfg.getXmax(), cfg.getYmax(), cfg.getDelta(), cfg.getOccThresh());
    }
}
