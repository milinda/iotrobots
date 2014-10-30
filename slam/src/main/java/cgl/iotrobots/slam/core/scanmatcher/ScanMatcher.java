package cgl.iotrobots.slam.core.scanmatcher;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.utils.Covariance3;
import cgl.iotrobots.slam.core.utils.OrientedPoint;
import cgl.iotrobots.slam.core.utils.Point;
import cgl.iotrobots.slam.core.utils.PointPair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

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
    enum Move{Front, Back, Left, Right, TurnLeft, TurnRight, Done}
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

    public void computeActiveArea(GMap map, OrientedPoint<Double> p, double[] readings){
        if (m_activeAreaComputed)
            return;
        Set<Point<Integer>> activeArea = new TreeSet<Point<Integer>>();
        OrientedPoint<Double> lp= new OrientedPoint<Double>(p.x, p.y, p.theta);
        lp.x+=Math.cos(p.theta)*m_laserPose.x-Math.sin(p.theta)*m_laserPose.y;
        lp.y+=Math.sin(p.theta)*m_laserPose.x+Math.cos(p.theta)*m_laserPose.y;
        lp.theta+=m_laserPose.theta;
        Point<Integer> p0 = map.world2map(lp);

        int readingIndex = 0;
        int angleIndex = 0;
        for (readingIndex = 0; readingIndex < m_laserBeams; readingIndex++, angleIndex++)
        if (m_generateMap){
            double d=readings[readingIndex];
            if (d>m_laserMaxRange)
                continue;
            if (d>m_usableRange)
                d=m_usableRange;

            Point<Double> phit=  new Point<Double>(d * Math.cos(lp.theta + m_laserAngles[angleIndex]) + lp.x,d * Math.sin(lp.theta+ m_laserAngles[angleIndex]) + lp.y);
            Point<Integer> p1 = map.world2map(phit);

            d += map.getDelta();
            //Point phit2=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
            //IntPoint p2=map.world2map(phit2);
            List<Point<Integer>> linePoints = new ArrayList<Point<Integer>>();
            GridLineTraversalLine line = new GridLineTraversalLine();
            line.points = linePoints;
            //GridLineTraversal::gridLine(p0, p2, &line);
            line.gridLine(p0, p1, line);
            for (int i=0; i<line.points.size()-1; i++){
                activeArea.add(map.getStorage().patchIndexes(linePoints[i]));
            }
            if (d<=m_usableRange){
                activeArea.add(map.getStorage().patchIndexes(p1));
                //activeArea.insert(map.storage().patchIndexes(p2));
            }
        } else {
            double r = readings[readingIndex];
            double angle = m_laserAngles[angleIndex];
            if (readings[readingIndex] > m_laserMaxRange ||readings[readingIndex]>m_usableRange) {
                continue;
            }
            Point<Double> phit= new Point<Double>(lp.x, lp.y);
            phit.x+=r*Math.cos(lp.theta+angle);
            phit.y+=r*Math.sin(lp.theta+angle);
            Point<Integer> p1=map.world2map(phit);
            assert(p1.x>=0 && p1.y>=0);
            Point<Integer> cp=map.getStorage().patchIndexes(p1);
            assert(cp.x>=0 && cp.y>=0);
            activeArea.add(cp);

        }
        //this allocates the unallocated cells in the active area of the map
        //cout << "activeArea::size() " << activeArea.size() << endl;
        map.getStorage().setActiveArea(activeArea, true);
        m_activeAreaComputed=true;
    }

    public void registerScan(GMap map, OrientedPoint<Double> p, double[] readings){
        if (!m_activeAreaComputed)
            computeActiveArea(map, p, readings);

        //this operation replicates the cells that will be changed in the registration operation
        map.getStorage().allocActiveArea();

        OrientedPoint lp=p;
        lp.x+=Math.cos(p.theta)*m_laserPose.x-Math.sin(p.theta)*m_laserPose.y;
        lp.y+=Math.sin(p.theta)*m_laserPose.x+Math.cos(p.theta)*m_laserPose.y;
        lp.theta+=m_laserPose.theta;
        IntPoint p0=map.world2map(lp);
        const double * angle=m_laserAngles;
        for (const double* r=readings; r<readings+m_laserBeams; r++, angle++)
        if (m_generateMap){
            double d=*r;
            if (d>m_laserMaxRange)
                continue;
            if (d>m_usableRange)
                d=m_usableRange;
            Point phit=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
            IntPoint p1=map.world2map(phit);

            d+=map.getDelta();
            //Point phit2=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
            //IntPoint p2=map.world2map(phit2);
            IntPoint linePoints[20000] ;
            GridLineTraversalLine line;
            line.points=linePoints;
            //GridLineTraversal::gridLine(p0, p2, &line);
            GridLineTraversal::gridLine(p0, p1, &line);
            for (int i=0; i<line.num_points-1; i++){
                map.cell(line.points[i]).update(false, Point(0,0));
            }
            if (d<=m_usableRange){
                map.cell(p1).update(true,phit);
                //	map.cell(p2).update(true,phit);
            }
        } else {
            if (*r>m_laserMaxRange||*r>m_usableRange) continue;
            Point phit=lp;
            phit.x+=*r*cos(lp.theta+*angle);
            phit.y+=*r*sin(lp.theta+*angle);
            map.cell(phit).update(true,phit);
        }
    }

    public double optimize(OrientedPoint<Double> _mean, Covariance3 _cov, GMap map, OrientedPoint<Double> init, double []readings){
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

    public double optimize(OrientedPoint<Double> pnew, GMap map, OrientedPoint<Double> init, double []readings) {
        double bestScore=-1;
        OrientedPoint<Double> currentPose=init;
        double currentScore=score(map, currentPose, readings);
        double adelta=m_optAngularDelta, ldelta=m_optLinearDelta;
        int refinement=0;
        int c_iterations=0;
        do{
            if (bestScore>=currentScore){
                refinement++;
                adelta*=.5;
                ldelta*=.5;
            }
            bestScore=currentScore;
            OrientedPoint<Double> bestLocalPose=currentPose;
            OrientedPoint<Double> localPose=currentPose;

            Move move=Move.Front;
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
                double localScore=score(map, localPose, readings);
                if (localScore>currentScore){
                    currentScore=localScore;
                    bestLocalPose=localPose;
                }
                c_iterations++;
            } while(move!=Move.Done);
            currentPose=bestLocalPose;
        }while (currentScore>bestScore || refinement<m_optRecursiveIterations);
        pnew.x=currentPose.x;
        pnew.y=currentPose.y;
        pnew.theta=currentPose.theta;
        return bestScore;
    }

    void setLaserParameters
            (int beams, double []angles, OrientedPoint<Double> lpose){
        m_laserPose=lpose;
        m_laserBeams=beams;
        m_laserAngles=new double[beams];
        System.arraycopy(angles, 0, m_laserAngles, 0, angles.length);
    }


    public void invalidateActiveArea(){
        m_activeAreaComputed=false;
    }

    public LikeliHood likelihood
            (GMap map, OrientedPoint<Double> p, double[] readings) {
        double _lmax;
        OrientedPoint<Double> _mean;
        Covariance3 _cov;
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

    public class LikeliHoodAndScore {
        public double s;
        public double l;
        public double c;

        public LikeliHoodAndScore(double s, double l, double c) {
            this.s = s;
            this.l = l;
            this.c = c;
        }
    }

    public LikeliHoodAndScore likelihoodAndScore(GMap map, OrientedPoint<Double> p, double []readings) {
        double l = 0;
        double s = 0;
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
        return new LikeliHoodAndScore(s, l, c);
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