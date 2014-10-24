package cgl.iotrobots.slam.core.scanmatcher;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.utils.Covariance3;
import cgl.iotrobots.slam.core.grid.HierarchicalArray2D;
import cgl.iotrobots.slam.core.utils.OrientedPoint;
import cgl.iotrobots.slam.core.utils.Point;
import cgl.iotrobots.slam.core.utils.PointPair;
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
    enum Move{Front, Back, Left, Right, TurnLeft, TurnRight, Done};
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

    double optimize(OrientedPoint<Double> _mean, Covariance3 _cov, Map<PointAccumulator, HierarchicalArray2D> map, OrientedPoint<Double> init, double []readings){
        List<ScoredMove> moveList =  new ArrayList<ScoredMove>();
        double bestScore=-1;
        OrientedPoint<Double> currentPose=init;
        ScoredMove sm= new ScoredMove(currentPose,0,0);
        int matched = likelihoodAndScore(sm.score, sm.likelihood, map, currentPose, readings);
        double currentScore=sm.score;
        moveList.add(sm);
        double adelta=m_optAngularDelta, ldelta=m_optLinearDelta;
        int refinement=0;

        do{
            if (bestScore>=currentScore){
                refinement++;
                adelta*=.5;
                ldelta*=.5;
            }
            bestScore=currentScore;
            OrientedPoint<Double> bestLocalPose=currentPose;
            OrientedPoint<Double> localPose=currentPose;

            Move move= Move.Front;
            do {
                localPose=currentPose;
                switch(move){
                    case Front:
                        localPose.x+=ldelta;
                        move=Move.Back;
                        break;
                    case Back:
                        localPose.x-=ldelta;
                        move=Move.Left;
                        break;
                    case Left:
                        localPose.y-=ldelta;
                        move=Move.Right;
                        break;
                    case Right:
                        localPose.y+=ldelta;
                        move=Move.TurnLeft;
                        break;
                    case TurnLeft:
                        localPose.theta+=adelta;
                        move=Move.TurnRight;
                        break;
                    case TurnRight:
                        localPose.theta-=adelta;
                        move=Move.Done;
                        break;
                    default:
                }
                double localScore = 0, localLikelihood = 0;
                //update the score
                matched=likelihoodAndScore(localScore, localLikelihood, map, localPose, readings);
                if (localScore>currentScore){
                    currentScore=localScore;
                    bestLocalPose=localPose;
                }
                sm.score=localScore;
                sm.likelihood=localLikelihood;
                sm.pose=localPose;
                moveList.add(sm);
                //update the move list
            } while(move!=Move.Done);
            currentPose=bestLocalPose;
            //here we look for the best move;
        }while (currentScore>bestScore || refinement<m_optRecursiveIterations);

        //normalize the likelihood
        double lmin=1e9;
        double lmax=-1e9;
        for (ScoredMove it : moveList){
            lmin=it.likelihood<lmin?it.likelihood:lmin;
            lmax=it.likelihood>lmax?it.likelihood:lmax;
        }
        for (ScoredMove it : moveList){
            it.likelihood=Math.exp(it.likelihood - lmax);
        }
        //compute the mean
        OrientedPoint<Double> mean = new OrientedPoint<Double>(0.0,0.0,0.0);
        double lacc=0;
        for (ScoredMove it : moveList){
            mean = OrientedPoint.plus(mean, OrientedPoint.mulN(it.pose, it.likelihood));
            lacc+=it.likelihood;
        }
        mean= OrientedPoint.mulN(mean, (1./lacc));
        //OrientedPoint delta=mean-currentPose;
        //cout << "delta.x=" << delta.x << " delta.y=" << delta.y << " delta.theta=" << delta.theta << endl;
        Covariance3 cov= new Covariance3(0.,0.,0.,0.,0.,0.);
        for (ScoredMove it:moveList){
            OrientedPoint<Double> delta= OrientedPoint.minus(it.pose, mean);
            delta.theta=Math.atan2(Math.sin(delta.theta), Math.cos(delta.theta));
            cov.xx+=delta.x*delta.x*it.likelihood;
            cov.yy+=delta.y*delta.y*it.likelihood;
            cov.tt+=delta.theta*delta.theta*it.likelihood;
            cov.xy+=delta.x*delta.y*it.likelihood;
            cov.xt+=delta.x*delta.theta*it.likelihood;
            cov.yt+=delta.y*delta.theta*it.likelihood;
        }
        cov.xx/=lacc;
        cov.xy/=lacc;
        cov.xt/=lacc;
        cov.yy/=lacc;
        cov.yt/=lacc;
        cov.tt/=lacc;

        _mean=currentPose;
        _cov=cov;
        return bestScore;
    }

    void setLaserParameters
            (int beams, double angles[], OrientedPoint<Double> lpose){
        m_laserPose= lpose;
        m_laserBeams=beams;
        m_laserAngles=new double[beams];
        m_laserAngles = angles;
    }

    public LikeliHood likelihood
            (double _lmax, OrientedPoint<Double> _mean, Covariance3 _cov, GMap map, OrientedPoint<Double> p, double[] readings) {
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

    public int likelihoodAndScore(double s, double l, GMap map, OrientedPoint<Double> p, double []readings) {
        l = 0;
        s = 0;
        int angleIndex = m_initialBeamsSkip;
        OrientedPoint<Double> lp = new OrientedPoint<Double>(p.x, p.y, p.theta);

        lp.x += Math.cos(p.theta) * m_laserPose.x - Math.sin(p.theta) * m_laserPose.y;
        lp.y += Math.sin(p.theta) * m_laserPose.x + Math.cos(p.theta) * m_laserPose.y;
        lp.theta += m_laserPose.theta;
        double noHit = nullLikelihood / (m_likelihoodSigma);
        int skip = 0;
        int c = 0;
        double freeDelta = map.getDelta()*m_freeCellRatio;
        for (int rIndex = m_initialBeamsSkip; rIndex < readings.length; rIndex++, angleIndex++) {
            skip++;
            skip=skip>m_likelihoodSkip?0:skip;
            if (readings[rIndex]>m_usableRange) continue;
            if (skip != 0) continue;
            Point<Double> phit = new Point<Double>(lp.x, lp.y);
            phit.x+=readings[rIndex]*Math.cos(lp.theta +m_laserAngles[angleIndex]);
            phit.y+=readings[rIndex]*Math.sin(lp.theta +m_laserAngles[angleIndex]);
            Point<Integer> iphit=map.world2map(phit);
            Point<Double> pfree = new Point<Double>(lp.x, lp.y);
            pfree.x+=(readings[rIndex]-freeDelta)*Math.cos(lp.theta +m_laserAngles[angleIndex]);
            pfree.y+=(readings[rIndex]-freeDelta)*Math.sin(lp.theta +m_laserAngles[angleIndex]);
            pfree.x  = pfree.x - phit.x;
            pfree.y  = pfree.y - phit.y;

            Point<Integer> ipfree = map.world2map(pfree);
            boolean found=false;
            Point<Double> bestMu = new Point<Double>(0.0, 0.0);
            for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
                for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
                    Point<Integer> pr = new Point<Integer>(iphit.x + xx, iphit.y + yy);
                    Point<Integer> pf = new Point<Integer>(pr.x + ipfree.x, pr.y + ipfree.y);
                    //AccessibilityState s=map.storage().cellState(pr);
                    //if (s&Inside && s&Allocated){
                    PointAccumulator cell = map.cell(pr);
                    PointAccumulator fcell = map.cell(pf);
                    if (((double)cell )>m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
                        Point<Double> mu=phit-cell.mean();
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

    double icpStep(OrientedPoint<Double> pret, GMap map, OrientedPoint<Double> p, double []readings) {
        int angleIndex = m_initialBeamsSkip;
        OrientedPoint<Double> lp= new OrientedPoint<Double>(p.x, p.y, p.theta);

        lp.x+=Math.cos(p.theta)*m_laserPose.x-Math.sin(p.theta)*m_laserPose.y;
        lp.y+=Math.sin(p.theta)*m_laserPose.x+Math.cos(p.theta)*m_laserPose.y;
        lp.theta+=m_laserPose.theta;
        int skip=0;
        double freeDelta=map.getDelta()*m_freeCellRatio;
        List<PointPair<Double>> pairs = new ArrayList<PointPair<Double>>();

        for (int rIndex = m_initialBeamsSkip; rIndex < readings.length; rIndex++, angleIndex++) {
            skip++;
            skip = skip > m_likelihoodSkip ? 0 : skip;
            if (readings[rIndex] > m_usableRange || readings[rIndex] == 0.0) continue;
            if (skip != 0) continue;
            Point<Double> phit = new Point<Double>(lp.x, lp.y);

            phit.x += readings[rIndex] * Math.cos(lp.theta + m_laserAngles[angleIndex]);
            phit.y += readings[rIndex] * Math.sin(lp.theta + m_laserAngles[angleIndex]);
            Point<Integer> iphit = map.world2map(phit);

            Point<Double> pfree = new Point<Double>(lp.x, lp.y);
            pfree.x += (readings[rIndex] - map.getDelta() * freeDelta) * Math.cos(lp.theta + m_laserAngles[angleIndex]);
            pfree.y += (readings[rIndex] - map.getDelta() * freeDelta) * Math.sin(lp.theta + m_laserAngles[angleIndex]);
            pfree.x  = pfree.x - phit.x;
            pfree.y  = pfree.y - phit.y;

            Point<Integer> ipfree = map.world2map(pfree);
            boolean found = false;
            Point<Double> bestMu = new Point<Double>(0., 0.);
            Point<Double> bestCell = new Point<Double>(0., 0.);
            for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++)
                for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++) {
                    Point<Integer> pr = new Point<Integer>(iphit.x + xx, iphit.y + yy);
                    Point<Integer> pf = new Point<Integer>(pr.x + ipfree.x, pr.y + ipfree.y);
                    //AccessibilityState s=map.storage().cellState(pr);
                    //if (s&Inside && s&Allocated){
                    PointAccumulator cell = (PointAccumulator) map.cell(pr);
                    PointAccumulator fcell = (PointAccumulator) map.cell(pf);

                    if (((double) cell) > m_fullnessThreshold && ((double) fcell) < m_fullnessThreshold) {
                        Point<Double> mu = phit - cell.mean();

                        if (!found) {
                            bestMu = mu;
                            bestCell = cell.mean();
                            found = true;
                        } else if ((mu * mu) < (bestMu * bestMu)) {
                            bestMu = mu;
                            bestCell = cell.mean();
                        }

                    }
                    //}
                }
            if (found) {
                pairs.add(new PointPair<Double>(phit, bestCell));
            }
        }

        OrientedPoint<Double> result = new OrientedPoint<Double>(0.0,0.0,0.0);
        //double icpError=icpNonlinearStep(result,pairs);
        LOG.error("result(" + pairs.size() + ")=" + result.x + " " + result.y + " " + result.theta);
        pret.x=p.x+result.x;
        pret.y=p.y+result.y;
        pret.theta=p.theta+result.theta;
        pret.theta=Math.atan2(Math.sin(pret.theta), Math.cos(pret.theta));
        return score(map, p, readings);
    }

    double score(GMap map, OrientedPoint<Double> p, double[] readings) {
        double s = 0;
        int angleIndex = m_initialBeamsSkip;
        OrientedPoint<Double> lp = new OrientedPoint<Double>(p.x, p.y, p.theta);
        lp.x += Math.cos(p.theta) * m_laserPose.x - Math.sin(p.theta) * m_laserPose.y;
        lp.y += Math.sin(p.theta) * m_laserPose.x + Math.cos(p.theta) * m_laserPose.y;
        lp.theta += m_laserPose.theta;
        int skip = 0;
        double freeDelta = map.getDelta() * m_freeCellRatio;
        for (int rIndex = m_initialBeamsSkip; rIndex < readings.length; rIndex++, angleIndex++) {
            skip++;
            skip = skip > m_likelihoodSkip ? 0 : skip;
            if (skip != 0 || readings[rIndex] > m_usableRange || readings[rIndex] == 0.0) continue;
            Point<Double> phit = new Point<Double>(lp.x, lp.y);
            phit.x += readings[rIndex] * Math.cos(lp.theta + m_laserAngles[angleIndex]);
            phit.y += readings[rIndex] * Math.sin(lp.theta + m_laserAngles[angleIndex]);
            Point<Integer> iphit = map.world2map(phit);
            Point<Double> pfree = new Point<Double>(lp.x, lp.y);
            pfree.x += ( readings[rIndex] - map.getDelta() * freeDelta)*Math.cos(lp.theta + m_laserAngles[angleIndex]);
            pfree.y += ( readings[rIndex] - map.getDelta() * freeDelta)*Math.sin(lp.theta + m_laserAngles[angleIndex]);
            pfree.x  = pfree.x - phit.x;
            pfree.y  = pfree.y - phit.y;
            Point<Integer> ipfree = map.world2map(pfree);
            boolean found = false;
            Point<Double> bestMu = new Point<Double>(0., 0.);
            for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++)
                for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++) {
                    Point<Integer> pr = new Point<Integer>(iphit.x + xx, iphit.y + yy);
                    Point<Integer> pf = new Point<Integer>(pr.x + ipfree.x, pr.y + ipfree.y);
                    //AccessibilityState s=map.storage().cellState(pr);
                    //if (s&Inside && s&Allocated){
                    PointAccumulator cell = map.cell(pr);
                    PointAccumulator fcell = map.cell(pf);
                    if (((double) cell) > m_fullnessThreshold && ((double) fcell) < m_fullnessThreshold) {
                        Point<Double> mu = phit - cell.mean();
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

        private ScoredMove(OrientedPoint<Double> pose, double score, double likelihood) {
            this.pose = pose;
            this.score = score;
            this.likelihood = likelihood;
        }
    }
}