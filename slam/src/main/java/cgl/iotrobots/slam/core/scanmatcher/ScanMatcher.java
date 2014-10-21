package cgl.iotrobots.slam.core.scanmatcher;

import cgl.iotrobots.slam.core.utils.Covariance3;
import cgl.iotrobots.slam.core.grid.HierarchicalArray2D;
import cgl.iotrobots.slam.core.utils.OrientedPoint;
import cgl.iotrobots.slam.core.utils.Point;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class ScanMatcher {
    private static Logger LOG = LoggerFactory.getLogger(ScanMatcher.class);

    public static final int LASER_MAXBEAMS = 2048;
    public static final double nullLikelihood=-1.;

    double m_laserMaxRange;
    double m_usableRange;
    double m_gaussianSigma;
    double m_likelihoodSigma;
    int m_kernelSize;
    double m_optAngularDelta;
    double m_optLinearDelta;
    int m_optRecursiveIterations;
    int m_likelihoodSkip;
    double m_llsamplerange;
    double m_lasamplerange;
    double m_llsamplestep;
    double m_lasamplestep;
    boolean m_generateMap;
    boolean m_enlargeStep;
    OrientedPoint<Double> m_laserPose;
    double m_fullnessThreshold;
    double m_angularOdometryReliability;
    double m_linearOdometryReliability;
    double m_freeCellRatio;
    int m_initialBeamsSkip;

    boolean m_activeAreaComputed;

    /**
     * laser parameters
     */
    int m_laserBeams;
    double m_laserAngles[] = new double[LASER_MAXBEAMS];

    public void setMatchingParameters
            (double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma, int likelihoodSkip) {
        m_usableRange = urange;
        m_laserMaxRange = range;
        m_kernelSize = kernsize;
        m_optLinearDelta = lopt;
        m_optAngularDelta = aopt;
        m_optRecursiveIterations = iterations;
        m_gaussianSigma = sigma;
        m_likelihoodSigma = likelihoodSigma;
        m_likelihoodSkip = likelihoodSkip;
    }

    public LikeliHood likelihood
            (double _lmax, OrientedPoint<Double> _mean, Covariance3 _cov, Map<PointAccumulator, HierarchicalArray2D> map, OrientedPoint<Double> p, double[] readings) {
        List<ScoredMove> moveList = new ArrayList<ScoredMove>();

        for (double xx = -m_llsamplerange; xx <= m_llsamplerange; xx += m_llsamplestep) {
            for (double yy = -m_llsamplerange; yy <= m_llsamplerange; yy += m_llsamplestep) {
                for (double tt = -m_lasamplerange; tt <= m_lasamplerange; tt += m_lasamplestep) {
                    OrientedPoint<Double> rp = new OrientedPoint<Double>(p.x, p.y, p.theta);
                    rp.x += xx;
                    rp.y += yy;
                    rp.theta += tt;

                    ScoredMove sm = new ScoredMove();
                    sm.pose = rp;

                    likelihoodAndScore(sm.score, sm.likelihood, map, rp, readings);
                    moveList.add(sm);
                }
            }
        }

        //normalize the likelihood
        double lmax = -1e9;
        double lcum = 0;
        for (ScoredMove it : moveList) {
            lmax = it.likelihood > lmax ? it.likelihood : lmax;
        }
        for (ScoredMove it : moveList) {
            lcum += Math.exp(it.likelihood - lmax);
            it.likelihood = Math.exp(it.likelihood - lmax);
        }

        OrientedPoint<Double> mean = new OrientedPoint<Double>(0.0, 0.0, 0.0);
        for (ScoredMove it : moveList) {
            double x = mean.x + it.pose.x * it.likelihood;
            double y = mean.y + it.pose.y * it.likelihood;
            double theta = mean.theta + it.pose.theta * it.likelihood;
            mean.x = x;
            mean.y = y;
            mean.theta = theta;
        }
        mean.x = mean.x * (1. / lcum);
        mean.y = mean.y * (1. / lcum);
        mean.theta = mean.theta * (1. / lcum);

        Covariance3 cov = new Covariance3(0.,0.,0.,0.,0.,0.);
        for (ScoredMove it : moveList){
            OrientedPoint<Double> delta = OrientedPoint.minus(it.pose, mean);
            delta.theta=Math.atan2(Math.sin(delta.theta), Math.cos(delta.theta));
            cov.xx+=delta.x*delta.x*it.likelihood;
            cov.yy+=delta.y*delta.y*it.likelihood;
            cov.tt+=delta.theta*delta.theta*it.likelihood;
            cov.xy+=delta.x*delta.y*it.likelihood;
            cov.xt+=delta.x*delta.theta*it.likelihood;
            cov.yt+=delta.y*delta.theta*it.likelihood;
        }
        cov.xx /=lcum;
        cov.xy/=lcum;
        cov.xt/=lcum;
        cov.yy/=lcum;
        cov.yt/=lcum;
        cov.tt/=lcum;

        return new LikeliHood(lmax, mean, cov, Math.log(lcum)+lmax);
    }

    public int likelihoodAndScore(double s, double l, Map<PointAccumulator, HierarchicalArray2D> map, OrientedPoint<Double> p, double []readings) {
        l = 0;
        s = 0;
        double angle = m_laserAngles + m_initialBeamsSkip;
        OrientedPoint<Double> lp = new OrientedPoint<Double>(p.x, p.y, p.theta);

        lp.x += Math.cos(p.theta) * m_laserPose.x - Math.sin(p.theta) * m_laserPose.y;
        lp.y += Math.sin(p.theta) * m_laserPose.x + Math.cos(p.theta) * m_laserPose.y;
        lp.theta += m_laserPose.theta;
        double noHit = nullLikelihood / (m_likelihoodSigma);
        int skip = 0;
        int c = 0;
        double freeDelta=map.getDelta()*m_freeCellRatio;
        for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
            skip++;
            skip=skip>m_likelihoodSkip?0:skip;
            if (*r>m_usableRange) continue;
            if (skip) continue;
            Point phit=lp;
            phit.x+=*r*cos(lp.theta+*angle);
            phit.y+=*r*sin(lp.theta+*angle);
            IntPoint iphit=map.world2map(phit);
            Point pfree=lp;
            pfree.x+=(*r-freeDelta)*cos(lp.theta+*angle);
            pfree.y+=(*r-freeDelta)*sin(lp.theta+*angle);
            pfree=pfree-phit;
            IntPoint ipfree=map.world2map(pfree);
            boolean found=false;
            Point<Double> bestMu = new Point<Double>(0.0, 0.0);
            for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
                for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
                    Point<Integer> pr= iphit + IntPoint(xx,yy);
                    Point<Integer> pf= pr+ipfree;
                    //AccessibilityState s=map.storage().cellState(pr);
                    //if (s&Inside && s&Allocated){
                    PointAccumulator cell = map.cell(pr);
                    PointAccumulator fcell = map.cell(pf);
                    if (((double)cell )>m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
                        Point mu=phit-cell.mean();
                        if (!found){
                            bestMu=mu;
                            found=true;
                        }else
                            bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
                    }
                    //}
                }
            if (found){
                s+=Math.exp(-1.0 / m_gaussianSigma * bestMu * bestMu);
                c++;
            }
            if (skip != 0){
                double f=(-1./m_likelihoodSigma)*(bestMu*bestMu);
                l+=(found)?f:noHit;
            }
        }
        return c;
    }

    double icpStep(OrientedPoint<Double> pret, Map<PointAccumulator, HierarchicalArray2D> map, OrientedPoint p, double []readings) {
        const double * angle=m_laserAngles+m_initialBeamsSkip;
        OrientedPoint lp=p;
        lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
        lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
        lp.theta+=m_laserPose.theta;
        unsigned int skip=0;
        double freeDelta=map.getDelta()*m_freeCellRatio;
        std::list<PointPair> pairs;

        for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
            skip++;
            skip=skip>m_likelihoodSkip?0:skip;
            if (*r>m_usableRange||*r==0.0) continue;
            if (skip) continue;
            Point phit=lp;
            phit.x+=*r*cos(lp.theta+*angle);
            phit.y+=*r*sin(lp.theta+*angle);
            IntPoint iphit=map.world2map(phit);
            Point pfree=lp;
            pfree.x+=(*r-map.getDelta()*freeDelta)*cos(lp.theta+*angle);
            pfree.y+=(*r-map.getDelta()*freeDelta)*sin(lp.theta+*angle);
            pfree=pfree-phit;
            IntPoint ipfree=map.world2map(pfree);
            bool found=false;
            Point bestMu(0.,0.);
            Point bestCell(0.,0.);
            for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
                for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
                    IntPoint pr=iphit+IntPoint(xx,yy);
                    IntPoint pf=pr+ipfree;
                    //AccessibilityState s=map.storage().cellState(pr);
                    //if (s&Inside && s&Allocated){
                    const PointAccumulator& cell=map.cell(pr);
                    const PointAccumulator& fcell=map.cell(pf);
                    if (((double)cell )> m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
                        Point mu=phit-cell.mean();
                        if (!found){
                            bestMu=mu;
                            bestCell=cell.mean();
                            found=true;
                        }else
                        if((mu*mu)<(bestMu*bestMu)){
                            bestMu=mu;
                            bestCell=cell.mean();
                        }

                    }
                    //}
                }
            if (found){
                pairs.push_back(std::make_pair(phit, bestCell));
                //std::cerr << "(" << phit.x-bestCell.x << "," << phit.y-bestCell.y << ") ";
            }
            //std::cerr << std::endl;
        }

        OrientedPoint result(0,0,0);
        //double icpError=icpNonlinearStep(result,pairs);
        std::cerr << "result(" << pairs.size() << ")=" << result.x << " " << result.y << " " << result.theta << std::endl;
        pret.x=p.x+result.x;
        pret.y=p.y+result.y;
        pret.theta=p.theta+result.theta;
        pret.theta=atan2(sin(pret.theta), cos(pret.theta));
        return score(map, p, readings);
    }

    double score(Map<PointAccumulator, HierarchicalArray2D> map, OrientedPoint<Double> p, double[] readings) {
        double s = 0;
        int angleIndex = m_initialBeamsSkip;
        OrientedPoint<Double> lp = new OrientedPoint<Double>(p.x, p.y, p.theta);
        lp.x += Math.cos(p.theta) * m_laserPose.x - Math.sin(p.theta) * m_laserPose.y;
        lp.y += Math.sin(p.theta) * m_laserPose.x + Math.cos(p.theta) * m_laserPose.y;
        lp.theta += m_laserPose.theta;
        int skip = 0;
        double freeDelta = map.getDelta() * m_freeCellRatio;
        for (int rIndex = m_initialBeamsSkip; rIndex < m_laserBeams; rIndex++, angleIndex++) {
            skip++;
            skip = skip > m_likelihoodSkip ? 0 : skip;
            if (skip != 0 || readings[rIndex] > m_usableRange || readings[rIndex] == 0.0) continue;
            Point<Double> phit = new Point<Double>(lp.x, lp.y);
            phit.x += readings[rIndex] * Math.cos(lp.theta + m_laserAngles[angleIndex]);
            phit.y += readings[rIndex] * Math.sin(lp.theta + m_laserAngles[angleIndex]);
            IntPoint iphit = map.world2map(phit);
            Point pfree = lp;
            pfree.x += ( * rIndex - map.getDelta() * freeDelta)*Math.cos(lp.theta + m_laserAngles[angleIndex]);
            pfree.y += ( * rIndex - map.getDelta() * freeDelta)*Math.sin(lp.theta + m_laserAngles[angleIndex]);
            pfree = pfree - phit;
            IntPoint ipfree = map.world2map(pfree);
            boolean found = false;
            Point<Double> bestMu = new Point<Double>(0., 0.);
            for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++)
                for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++) {
                    IntPoint pr = iphit + IntPoint(xx, yy);
                    IntPoint pf = pr + ipfree;
                    //AccessibilityState s=map.storage().cellState(pr);
                    //if (s&Inside && s&Allocated){
                    const PointAccumulator & cell = map.cell(pr);
                    const PointAccumulator & fcell = map.cell(pf);
                    if (((double) cell) > m_fullnessThreshold && ((double) fcell) < m_fullnessThreshold) {
                        Point mu = phit - cell.mean();
                        if (!found) {
                            bestMu = mu;
                            found = true;
                        } else
                            bestMu = (mu * mu) < (bestMu * bestMu) ? mu : bestMu;
                    }
                    //}
                }
            if (found) {
                s += Math.exp(-1. / m_gaussianSigma * bestMu * bestMu);
            }
        }
        return s;
    }

    private class ScoredMove {
        OrientedPoint<Double> pose;
        double score;
        double likelihood;
    }
}