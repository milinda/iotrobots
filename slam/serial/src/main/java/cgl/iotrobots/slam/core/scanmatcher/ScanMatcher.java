package cgl.iotrobots.slam.core.scanmatcher;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.utils.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class ScanMatcher {
    private static Logger LOG = LoggerFactory.getLogger(ScanMatcher.class);

    public static final int LASER_MAXBEAMS = 2048;
    public static final double nullLikelihood = -.5;

    double m_laserMaxRange;
    double m_usableRange;
    double m_gaussianSigma;
    double m_likelihoodSigma;
    int m_kernelSize;
    double m_optAngularDelta;
    double m_optLinearDelta;
    int m_optRecursiveIterations = 3;
    int m_likelihoodSkip;
    double m_llsamplerange;
    double m_lasamplerange;
    double m_llsamplestep;
    double m_lasamplestep;
    boolean m_generateMap;
    double m_enlargeStep = 10;
    DoubleOrientedPoint m_laserPose;
    double m_fullnessThreshold = 0.1;
    double m_angularOdometryReliability = 0;
    double m_linearOdometryReliability = 0;
    double m_freeCellRatio = Math.sqrt(2.0);
    int m_initialBeamsSkip = 0;

    boolean m_activeAreaComputed = false;

    enum Move {Front, Back, Left, Right, TurnLeft, TurnRight, Done}

    /**
     * laser parameters
     */
    int m_laserBeams;
    double m_laserAngles[] = new double[LASER_MAXBEAMS];

//    List<IntPoint> m_linePoints = new ArrayList<IntPoint>();

    public void setlaserMaxRange(double m_laserMaxRange) {
        this.m_laserMaxRange = m_laserMaxRange;
    }

    public void setusableRange(double m_usableRange) {
        this.m_usableRange = m_usableRange;
    }

    public void setgaussianSigma(double m_gaussianSigma) {
        this.m_gaussianSigma = m_gaussianSigma;
    }

    public void setlikelihoodSigma(double m_likelihoodSigma) {
        this.m_likelihoodSigma = m_likelihoodSigma;
    }

    public void setkernelSize(int m_kernelSize) {
        this.m_kernelSize = m_kernelSize;
    }

    public void setoptAngularDelta(double m_optAngularDelta) {
        this.m_optAngularDelta = m_optAngularDelta;
    }

    public void setoptLinearDelta(double m_optLinearDelta) {
        this.m_optLinearDelta = m_optLinearDelta;
    }

    public void setoptRecursiveIterations(int m_optRecursiveIterations) {
        this.m_optRecursiveIterations = m_optRecursiveIterations;
    }

    public void setlikelihoodSkip(int m_likelihoodSkip) {
        this.m_likelihoodSkip = m_likelihoodSkip;
    }

    public void setllsamplerange(double m_llsamplerange) {
        this.m_llsamplerange = m_llsamplerange;
    }

    public void setlasamplerange(double m_lasamplerange) {
        this.m_lasamplerange = m_lasamplerange;
    }

    public void setllsamplestep(double m_llsamplestep) {
        this.m_llsamplestep = m_llsamplestep;
    }

    public void setlasamplestep(double m_lasamplestep) {
        this.m_lasamplestep = m_lasamplestep;
    }

    public void setgenerateMap(boolean m_generateMap) {
        this.m_generateMap = m_generateMap;
    }

    public void setenlargeStep(double m_enlargeStep) {
        this.m_enlargeStep = m_enlargeStep;
    }

    public void setlaserPose(DoubleOrientedPoint m_laserPose) {
        this.m_laserPose = m_laserPose;
    }

    public void setfullnessThreshold(double m_fullnessThreshold) {
        this.m_fullnessThreshold = m_fullnessThreshold;
    }

    public void setangularOdometryReliability(double m_angularOdometryReliability) {
        this.m_angularOdometryReliability = m_angularOdometryReliability;
    }

    public void setlinearOdometryReliability(double m_linearOdometryReliability) {
        this.m_linearOdometryReliability = m_linearOdometryReliability;
    }

    public void setfreeCellRatio(double m_freeCellRatio) {
        this.m_freeCellRatio = m_freeCellRatio;
    }

    public void setinitialBeamsSkip(int m_initialBeamsSkip) {
        this.m_initialBeamsSkip = m_initialBeamsSkip;
    }

    public void setactiveAreaComputed(boolean m_activeAreaComputed) {
        this.m_activeAreaComputed = m_activeAreaComputed;
    }

    public void setlaserBeams(int m_laserBeams) {
        this.m_laserBeams = m_laserBeams;
    }

    public void setlaserAngles(double[] m_laserAngles) {
        this.m_laserAngles = m_laserAngles;
    }

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

    public void computeActiveArea(GMap map, DoubleOrientedPoint p, double[] readings) {
        if (m_activeAreaComputed)
            return;
//        Set<Point<Integer>> activeArea = new TreeSet<Point<Integer>>(new PointComparator());
        Set<IntPoint> activeArea = new HashSet<IntPoint>();
        DoubleOrientedPoint lp = new DoubleOrientedPoint(p.x, p.y, p.theta);
        lp.x += Math.cos(p.theta) * m_laserPose.x - Math.sin(p.theta) * m_laserPose.y;
        lp.y += Math.sin(p.theta) * m_laserPose.x + Math.cos(p.theta) * m_laserPose.y;
        lp.theta += m_laserPose.theta;
        IntPoint p0 = map.world2map(lp);

        DoublePoint min = map.map2world(new IntPoint(0, 0));
        DoublePoint max = map.map2world(new IntPoint(map.getMapSizeX() - 1, map.getMapSizeY() - 1));

        if (lp.x < min.x) min.x = lp.x;
        if (lp.y < min.y) min.y = lp.y;
        if (lp.x > max.x) max.x = lp.x;
        if (lp.y > max.y) max.y = lp.y;
//        System.out.println("Computing active area");
        int readingIndex;
        int angleIndex = m_initialBeamsSkip;
        for (readingIndex = m_initialBeamsSkip; readingIndex < m_laserBeams; readingIndex++, angleIndex++) {
            double r = readings[readingIndex];
            double angle = m_laserAngles[angleIndex];

            if (r > m_laserMaxRange || r == 0.0 || r > Double.MAX_VALUE) continue;
            double d = r > m_usableRange ? m_usableRange : r;
            DoublePoint phit = new DoublePoint(lp.x, lp.y);
            phit.x += d * Math.cos(lp.theta + angle);
            phit.y += d * Math.sin(lp.theta + angle);
            if (phit.x < min.x) min.x = phit.x;
            if (phit.y < min.y) min.y = phit.y;
            if (phit.x > max.x) max.x = phit.x;
            if (phit.y > max.y) max.y = phit.y;
        }

        if (!map.isInsideD(min) || !map.isInsideD(max)) {
            DoublePoint lmin = map.map2world(new IntPoint(0, 0));
            DoublePoint lmax = map.map2world(new IntPoint(map.getMapSizeX() - 1, map.getMapSizeY() - 1));
            //cerr << "CURRENT MAP " << lmin.x << " " << lmin.y << " " << lmax.x << " " << lmax.y << endl;
            //cerr << "BOUNDARY OVERRIDE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
            min.x = (min.x >= lmin.x) ? lmin.x : min.x - m_enlargeStep;
            max.x = (max.x <= lmax.x) ? lmax.x : max.x + m_enlargeStep;
            min.y = (min.y >= lmin.y) ? lmin.y : min.y - m_enlargeStep;
            max.y = (max.y <= lmax.y) ? lmax.y : max.y + m_enlargeStep;
            map.resize(min.x, min.y, max.x, max.y);
            //cerr << "RESIZE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
        }

        readingIndex = m_initialBeamsSkip;
        angleIndex = m_initialBeamsSkip;
        for (readingIndex = m_initialBeamsSkip; readingIndex < m_laserBeams; readingIndex++, angleIndex++)
            if (m_generateMap) {
                double d = readings[readingIndex];
                if (d > m_laserMaxRange || d == 0.0 || d > Double.MAX_VALUE)
                    continue;
                if (d > m_usableRange)
                    d = m_usableRange;

                DoublePoint phit = new DoublePoint(d * Math.cos(lp.theta + m_laserAngles[angleIndex]) + lp.x, d * Math.sin(lp.theta + m_laserAngles[angleIndex]) + lp.y);
                p0 = map.world2map(lp);
                IntPoint p1 = map.world2map(phit);

                d += map.getDelta();
                //Point phit2=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
                //IntPoint p2=map.world2map(phit2);
                // List<IntPoint> linePoints = new ArrayList<IntPoint>();
                GridLineTraversalLine line = new GridLineTraversalLine();
                line.points = m_linePoints;
                //GridLineTraversal::gridLine(p0, p2, &line);
                GridLineTraversalLine.gridLine(p0, p1, line);
                for (int i = 0; i < line.numPoints - 1; i++) {
                    IntPoint patch = map.getStorage().patchIndexes(m_linePoints[i]);
                    //if (map.isInside(patch)) {
                    activeArea.add(patch);
                    //}
//                    System.out.println(patch.x + ", " + patch.y);
                }
                if (d <= m_usableRange) {
                    IntPoint patch = map.getStorage().patchIndexes(p1);
//                    System.out.println(patch.x + ", " + patch.y);
                    //if (map.isInside(patch)) {
                        activeArea.add(patch);
                    //}
                    //activeArea.insert(map.storage().patchIndexes(p2));
                }
            } else {
                double r = readings[readingIndex];
                double angle = m_laserAngles[angleIndex];
                if (readings[readingIndex] > m_laserMaxRange || readings[readingIndex] > m_usableRange) {
                    continue;
                }
                DoublePoint phit = new DoublePoint(lp.x, lp.y);
                phit.x += r * Math.cos(lp.theta + angle);
                phit.y += r * Math.sin(lp.theta + angle);
                IntPoint p1 = map.world2map(phit);
                assert (p1.x >= 0 && p1.y >= 0);
                IntPoint cp = map.getStorage().patchIndexes(p1);
                assert (cp.x >= 0 && cp.y >= 0);
                //if (map.isInside(cp)) {
                    activeArea.add(cp);
                //}
//                System.out.println(cp.x + ", " + cp.y);

            }
        //this allocates the unallocated cells in the active area of the map
        //cout << "activeArea::size() " << activeArea.size() << endl;
        map.getStorage().setActiveArea(activeArea, true);
        m_activeAreaComputed = true;
    }

    private static int count = 0;


    private IntPoint m_linePoints [] = new IntPoint[20000];

    public double registerScan(GMap map, DoubleOrientedPoint p, double[] readings) {
        if (!m_activeAreaComputed)
            computeActiveArea(map, p, readings);

        //this operation replicates the cells that will be changed in the registration operation
        map.getStorage().allocActiveArea();

        DoubleOrientedPoint lp = new DoubleOrientedPoint(p.x, p.y, p.theta);
        lp.x += Math.cos(p.theta) * m_laserPose.x - Math.sin(p.theta) * m_laserPose.y;
        lp.y += Math.sin(p.theta) * m_laserPose.x + Math.cos(p.theta) * m_laserPose.y;
        lp.theta += m_laserPose.theta;
        IntPoint p0 = map.world2map(lp);
        int readingIndex;
        int angleIndex = m_initialBeamsSkip;
        double esum = 0;
        for (readingIndex = m_initialBeamsSkip; readingIndex < m_laserBeams; readingIndex++, angleIndex++) {
            if (m_generateMap) {
                double d = readings[readingIndex];
                if (d > m_laserMaxRange)
                    continue;
                if (d > m_usableRange)
                    d = m_usableRange;
                DoublePoint phit = new DoublePoint(d * Math.cos(lp.theta + m_laserAngles[angleIndex]) + lp.x, d * Math.sin(lp.theta + m_laserAngles[angleIndex]) + lp.x);
                IntPoint p1 = map.world2map(phit);

                d += map.getDelta();
                //Point phit2=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
                //IntPoint p2=map.world2map(phit2);
//                List<IntPoint> linePoints = new ArrayList<IntPoint>(20000);
                GridLineTraversalLine line = new GridLineTraversalLine();
                line.points = m_linePoints;
                //GridLineTraversal::gridLine(p0, p2, &line);
                GridLineTraversalLine.gridLine(p0, p1, line);
                for (int i = 0; i < line.numPoints - 1; i++) {
                    PointAccumulator pa = (PointAccumulator) map.cell(line.points[i], false);
                    double e = -pa.entropy();
                    pa.update(false, new DoublePoint(0.0, 0.0));
                    e += pa.entropy();
                    esum += e;
                }
                if (d <= m_usableRange) {
                    PointAccumulator pa = (PointAccumulator) map.cell(p1, false);
                    pa.update(true, phit);
                    //	map.cell(p2).update(true,phit);
//                    count++;
//                    System.out.println(count + " " + pa.doubleValue());
                }


            } else {
                double r = readings[readingIndex];
                if (r > m_laserMaxRange || r > m_usableRange || r == 0) {
                    continue;
                }
                DoublePoint phit = new DoublePoint(lp.x, lp.y);
                phit.x += r * Math.cos(lp.theta + m_laserAngles[angleIndex]);
                phit.y += r * Math.sin(lp.theta + m_laserAngles[angleIndex]);
                IntPoint p1 = map.world2map(phit);
                PointAccumulator pa = (PointAccumulator) map.cell(p1, false);
                pa.update(true, phit);
            }
        }
        return esum;
    }

    public double icpOptimize(DoubleOrientedPoint pnew, GMap map, DoubleOrientedPoint init, double[] readings) {
        double currentScore;
        double sc = score(map, init, readings);
        DoubleOrientedPoint start = init;
        pnew.x = init.x;
        pnew.y = init.y;
        pnew.theta = init.theta;
        int iterations = 0;
        do {
            currentScore = sc;
            sc = icpStep(pnew, map, start, readings);
            //cerr << "pstart=" << start.x << " " <<start.y << " " << start.theta << endl;
            //cerr << "pret=" << pnew.x << " " <<pnew.y << " " << pnew.theta << endl;
            start = pnew;
            iterations++;
        } while (sc > currentScore);
        return currentScore;
    }

    public double optimize(DoubleOrientedPoint _mean, Covariance3 _cov, GMap map, DoubleOrientedPoint init, double[] readings) {
        List<ScoredMove> moveList = new ArrayList<ScoredMove>();
        double bestScore = -1;
        DoubleOrientedPoint currentPose = init;
        ScoredMove sm = new ScoredMove(currentPose, 0, 0);
        LikeliHoodAndScore ls = likelihoodAndScore(map, currentPose, readings);
        sm.likelihood = ls.l;
        sm.score = ls.s;
        double currentScore = sm.score;
        moveList.add(sm);
        double adelta = m_optAngularDelta, ldelta = m_optLinearDelta;
        int refinement = 0;

        do {
            if (bestScore >= currentScore) {
                refinement++;
                adelta *= .5;
                ldelta *= .5;
            }
            bestScore = currentScore;
            DoubleOrientedPoint bestLocalPose = currentPose;
            DoubleOrientedPoint localPose = currentPose;

            Move move = Move.Front;
            do {
                localPose = currentPose;
                switch (move) {
                    case Front:
                        localPose.x += ldelta;
                        move = Move.Back;
                        break;
                    case Back:
                        localPose.x -= ldelta;
                        move = Move.Left;
                        break;
                    case Left:
                        localPose.y -= ldelta;
                        move = Move.Right;
                        break;
                    case Right:
                        localPose.y += ldelta;
                        move = Move.TurnLeft;
                        break;
                    case TurnLeft:
                        localPose.theta += adelta;
                        move = Move.TurnRight;
                        break;
                    case TurnRight:
                        localPose.theta -= adelta;
                        move = Move.Done;
                        break;
                    default:
                }
                double localScore = 0, localLikelihood = 0;
                //update the score
                ls = likelihoodAndScore(map, localPose, readings);
                localLikelihood = ls.s;
                localLikelihood = ls.l;
                if (localScore > currentScore) {
                    currentScore = localScore;
                    bestLocalPose = localPose;
                }
                sm.score = localScore;
                sm.likelihood = localLikelihood;
                sm.pose = localPose;
                moveList.add(sm);
                //update the move list
            } while (move != Move.Done);
            currentPose = bestLocalPose;
            //here we look for the best move;
        } while (currentScore > bestScore || refinement < m_optRecursiveIterations);

        //normalize the likelihood
        double lmin = 1e9;
        double lmax = -1e9;
        for (ScoredMove it : moveList) {
            lmin = it.likelihood < lmin ? it.likelihood : lmin;
            lmax = it.likelihood > lmax ? it.likelihood : lmax;
        }
        for (ScoredMove it : moveList) {
            it.likelihood = Math.exp(it.likelihood - lmax);
        }
        //compute the mean
        DoubleOrientedPoint mean = new DoubleOrientedPoint(0.0, 0.0, 0.0);
        double lacc = 0;
        for (ScoredMove it : moveList) {
            mean = DoubleOrientedPoint.plus(mean, DoubleOrientedPoint.mulN(it.pose, it.likelihood));
            lacc += it.likelihood;
        }
        mean = DoubleOrientedPoint.mulN(mean, (1. / lacc));
        //OrientedPoint delta=mean-currentPose;
        //cout << "delta.x=" << delta.x << " delta.y=" << delta.y << " delta.theta=" << delta.theta << endl;
        Covariance3 cov = new Covariance3(0., 0., 0., 0., 0., 0.);
        for (ScoredMove it : moveList) {
            DoubleOrientedPoint delta = DoubleOrientedPoint.minus(it.pose, mean);
            delta.theta = Math.atan2(Math.sin(delta.theta), Math.cos(delta.theta));
            cov.xx += delta.x * delta.x * it.likelihood;
            cov.yy += delta.y * delta.y * it.likelihood;
            cov.tt += delta.theta * delta.theta * it.likelihood;
            cov.xy += delta.x * delta.y * it.likelihood;
            cov.xt += delta.x * delta.theta * it.likelihood;
            cov.yt += delta.y * delta.theta * it.likelihood;
        }
        cov.xx /= lacc;
        cov.xy /= lacc;
        cov.xt /= lacc;
        cov.yy /= lacc;
        cov.yt /= lacc;
        cov.tt /= lacc;

        _mean = currentPose;
        _cov = cov;
        return bestScore;
    }

    public double optimize(DoubleOrientedPoint pnew, GMap map, DoubleOrientedPoint init, double[] readings) {
        double bestScore = -1;
        DoubleOrientedPoint currentPose = new DoubleOrientedPoint(init.x, init.y, init.theta);
        double currentScore = score(map, currentPose, readings);
        double adelta = m_optAngularDelta, ldelta = m_optLinearDelta;
        int refinement = 0;
        int c_iterations = 0;
        do {
            if (bestScore >= currentScore) {
                refinement++;
                adelta *= .5;
                ldelta *= .5;
            }
            bestScore = currentScore;
            DoubleOrientedPoint bestLocalPose = new DoubleOrientedPoint(currentPose.x, currentPose.y, currentPose.theta);
            DoubleOrientedPoint localPose;

            Move move = Move.Front;
            do {
                localPose = new DoubleOrientedPoint(currentPose.x, currentPose.y, currentPose.theta);
                switch (move) {
                    case Front:
                        localPose.x += ldelta;
                        move = Move.Back;
                        break;
                    case Back:
                        localPose.x -= ldelta;
                        move = Move.Left;
                        break;
                    case Left:
                        localPose.y -= ldelta;
                        move = Move.Right;
                        break;
                    case Right:
                        localPose.y += ldelta;
                        move = Move.TurnLeft;
                        break;
                    case TurnLeft:
                        localPose.theta += adelta;
                        move = Move.TurnRight;
                        break;
                    case TurnRight:
                        localPose.theta -= adelta;
                        move = Move.Done;
                        break;
                    default:
                }

                double odo_gain = 1;
                if (m_angularOdometryReliability > 0.) {
                    double dth = init.theta - localPose.theta;
                    dth = Math.atan2(Math.sin(dth), Math.cos(dth));
                    dth *= dth;
                    odo_gain *= Math.exp(-m_angularOdometryReliability * dth);
                }
                if (m_linearOdometryReliability > 0.) {
                    double dx = init.x - localPose.x;
                    double dy = init.y - localPose.y;
                    double drho = dx * dx + dy * dy;
                    odo_gain *= Math.exp(-m_linearOdometryReliability * drho);
                }

                double localScore = odo_gain * score(map, localPose, readings);
                if (localScore > currentScore) {
                    currentScore = localScore;
                    bestLocalPose = new DoubleOrientedPoint(localPose.x, localPose.y, localPose.theta);
                }
                c_iterations++;
            } while (move != Move.Done);
            currentPose = new DoubleOrientedPoint(bestLocalPose.x, bestLocalPose.y, bestLocalPose.theta);
        } while (currentScore > bestScore || refinement < m_optRecursiveIterations);
        pnew.x = currentPose.x;
        pnew.y = currentPose.y;
        pnew.theta = currentPose.theta;
        return bestScore;
    }

    public void setLaserParameters
            (int beams, double[] angles, DoubleOrientedPoint lpose) {
        m_laserPose = lpose;
        m_laserBeams = beams;
        m_laserAngles = new double[beams];
        System.arraycopy(angles, 0, m_laserAngles, 0, angles.length);
    }


    public void invalidateActiveArea() {
        m_activeAreaComputed = false;
    }

    public LikeliHood likelihood
            (GMap map, DoubleOrientedPoint p, double[] readings) {
        double _lmax;
        DoubleOrientedPoint _mean;
        Covariance3 _cov;
        List<ScoredMove> moveList = new ArrayList<ScoredMove>();

        for (double xx = -m_llsamplerange; xx <= m_llsamplerange; xx += m_llsamplestep) {
            for (double yy = -m_llsamplerange; yy <= m_llsamplerange; yy += m_llsamplestep) {
                for (double tt = -m_lasamplerange; tt <= m_lasamplerange; tt += m_lasamplestep) {
                    DoubleOrientedPoint rp = new DoubleOrientedPoint(p.x, p.y, p.theta);
                    rp.x += xx;
                    rp.y += yy;
                    rp.theta += tt;

                    ScoredMove sm = new ScoredMove();
                    sm.pose = rp;

                    LikeliHoodAndScore ls = likelihoodAndScore(map, rp, readings);
                    sm.score = ls.s;
                    sm.likelihood = ls.l;

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

        DoubleOrientedPoint mean = new DoubleOrientedPoint(0.0, 0.0, 0.0);
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

        Covariance3 cov = new Covariance3(0., 0., 0., 0., 0., 0.);
        for (ScoredMove it : moveList) {
            DoubleOrientedPoint delta = DoubleOrientedPoint.minus(it.pose, mean);
            delta.theta = Math.atan2(Math.sin(delta.theta), Math.cos(delta.theta));
            cov.xx += delta.x * delta.x * it.likelihood;
            cov.yy += delta.y * delta.y * it.likelihood;
            cov.tt += delta.theta * delta.theta * it.likelihood;
            cov.xy += delta.x * delta.y * it.likelihood;
            cov.xt += delta.x * delta.theta * it.likelihood;
            cov.yt += delta.y * delta.theta * it.likelihood;
        }
        cov.xx /= lcum;
        cov.xy /= lcum;
        cov.xt /= lcum;
        cov.yy /= lcum;
        cov.yt /= lcum;
        cov.tt /= lcum;

        return new LikeliHood(lmax, mean, cov, Math.log(lcum));
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

    public LikeliHoodAndScore likelihoodAndScore(GMap map, DoubleOrientedPoint p, double[] readings) {
        double l = 0;
        double s = 0;
        int angleIndex = m_initialBeamsSkip;
        DoubleOrientedPoint lp = new DoubleOrientedPoint(p.x, p.y, p.theta);

        lp.x += Math.cos(p.theta) * m_laserPose.x - Math.sin(p.theta) * m_laserPose.y;
        lp.y += Math.sin(p.theta) * m_laserPose.x + Math.cos(p.theta) * m_laserPose.y;
        lp.theta += m_laserPose.theta;
        double noHit = nullLikelihood / (m_likelihoodSigma);
        int skip = 0;
        int c = 0;
        double freeDelta = map.getDelta() * m_freeCellRatio;
        for (int rIndex = m_initialBeamsSkip; rIndex < readings.length; rIndex++, angleIndex++) {
            skip++;
            skip = skip > m_likelihoodSkip ? 0 : skip;
            if (readings[rIndex] > m_usableRange) {
                continue;
            }
            if (skip != 0) {
                continue;
            }
            DoublePoint phit = new DoublePoint(lp.x, lp.y);
            phit.x += readings[rIndex] * Math.cos(lp.theta + m_laserAngles[angleIndex]);
            phit.y += readings[rIndex] * Math.sin(lp.theta + m_laserAngles[angleIndex]);
            IntPoint iphit = map.world2map(phit);
            DoublePoint pfree = new DoublePoint(lp.x, lp.y);
            pfree.x += (readings[rIndex] - freeDelta) * Math.cos(lp.theta + m_laserAngles[angleIndex]);
            pfree.y += (readings[rIndex] - freeDelta) * Math.sin(lp.theta + m_laserAngles[angleIndex]);
            pfree.x = pfree.x - phit.x;
            pfree.y = pfree.y - phit.y;

            IntPoint ipfree = map.world2map(pfree);
            boolean found = false;
            DoublePoint bestMu = new DoublePoint(0.0, 0.0);
            for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++)
                for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++) {
                    IntPoint pr = new IntPoint(iphit.x + xx, iphit.y + yy);
                    IntPoint pf = new IntPoint(pr.x + ipfree.x, pr.y + ipfree.y);
                    //AccessibilityState s=map.storage().cellState(pr);
                    //if (s&Inside && s&Allocated){
                    PointAccumulator cell = (PointAccumulator) map.cell(pr, true);
                    PointAccumulator fcell = (PointAccumulator) map.cell(pf, true);
                    if (cell.doubleValue() > m_fullnessThreshold && fcell.doubleValue() < m_fullnessThreshold) {
                        DoublePoint mu = DoublePoint.minus(phit, cell.mean());
                        if (!found) {
                            bestMu = mu;
                            found = true;
                        } else {
                            bestMu = (DoublePoint.mulD(mu, mu)) < DoublePoint.mulD(bestMu, bestMu) ? mu : bestMu;
                        }
                    }
                    //}
                }
            if (found) {
                s += Math.exp(-1.0 / m_gaussianSigma * DoublePoint.mulD(bestMu, bestMu));
                c++;
            }
            if (skip == 0) {
                double f = (-1./m_likelihoodSigma) * (DoublePoint.mulD(bestMu, bestMu));
                l += (found) ? f : noHit;
            }
        }
        return new LikeliHoodAndScore(s, l, c);
    }

    double icpStep(DoubleOrientedPoint pret, GMap map, DoubleOrientedPoint p, double[] readings) {
        int angleIndex = m_initialBeamsSkip;
        DoubleOrientedPoint lp = new DoubleOrientedPoint(p.x, p.y, p.theta);

        lp.x += Math.cos(p.theta) * m_laserPose.x - Math.sin(p.theta) * m_laserPose.y;
        lp.y += Math.sin(p.theta) * m_laserPose.x + Math.cos(p.theta) * m_laserPose.y;
        lp.theta += m_laserPose.theta;
        int skip = 0;
        double freeDelta = map.getDelta() * m_freeCellRatio;
        List<DoublePointPair> pairs = new ArrayList<DoublePointPair>();

        for (int rIndex = m_initialBeamsSkip; rIndex < readings.length; rIndex++, angleIndex++) {
            skip++;
            skip = skip > m_likelihoodSkip ? 0 : skip;
            if (readings[rIndex] > m_usableRange || readings[rIndex] == 0.0) continue;
            if (skip != 0) continue;
            DoublePoint phit = new DoublePoint(lp.x, lp.y);

            phit.x += readings[rIndex] * Math.cos(lp.theta + m_laserAngles[angleIndex]);
            phit.y += readings[rIndex] * Math.sin(lp.theta + m_laserAngles[angleIndex]);
            IntPoint iphit = map.world2map(phit);

            DoublePoint pfree = new DoublePoint(lp.x, lp.y);
            pfree.x += (readings[rIndex] - map.getDelta() * freeDelta) * Math.cos(lp.theta + m_laserAngles[angleIndex]);
            pfree.y += (readings[rIndex] - map.getDelta() * freeDelta) * Math.sin(lp.theta + m_laserAngles[angleIndex]);
            pfree.x = pfree.x - phit.x;
            pfree.y = pfree.y - phit.y;

            IntPoint ipfree = map.world2map(pfree);
            boolean found = false;
            DoublePoint bestMu = new DoublePoint(0., 0.);
            DoublePoint bestCell = new DoublePoint(0., 0.);
            for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++)
                for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++) {
                    IntPoint pr = new IntPoint(iphit.x + xx, iphit.y + yy);
                    IntPoint pf = new IntPoint(pr.x + ipfree.x, pr.y + ipfree.y);
                    //AccessibilityState s=map.storage().cellState(pr);
                    //if (s&Inside && s&Allocated){
                    PointAccumulator cell = (PointAccumulator) map.cell(pr, true);
                    PointAccumulator fcell = (PointAccumulator) map.cell(pf, true);

                    if (cell.doubleValue() > m_fullnessThreshold && fcell.doubleValue() < m_fullnessThreshold) {
                        DoublePoint mu = DoublePoint.minus(phit, cell.mean());
                        if (!found) {
                            bestMu = mu;
                            bestCell = cell.mean();
                            found = true;
                        } else if (DoublePoint.mulD(mu, mu) < DoublePoint.mulD(bestMu, bestMu)) {
                            bestMu = mu;
                            bestCell = cell.mean();
                        }
                    }
                    //}
                }
            if (found) {
                pairs.add(new DoublePointPair(phit, bestCell));
            }
        }

        DoubleOrientedPoint result = new DoubleOrientedPoint(0.0, 0.0, 0.0);
        //double icpError=icpNonlinearStep(result,pairs);
        LOG.error("result(" + pairs.size() + ")=" + result.x + " " + result.y + " " + result.theta);
        pret.x = p.x + result.x;
        pret.y = p.y + result.y;
        pret.theta = p.theta + result.theta;
        pret.theta = Math.atan2(Math.sin(pret.theta), Math.cos(pret.theta));
        return score(map, p, readings);
    }

    double score(GMap map, DoubleOrientedPoint p, double[] readings) {
        double s = 0;
        int angleIndex = m_initialBeamsSkip;
        DoubleOrientedPoint lp = new DoubleOrientedPoint(p.x, p.y, p.theta);
        lp.x += Math.cos(p.theta) * m_laserPose.x - Math.sin(p.theta) * m_laserPose.y;
        lp.y += Math.sin(p.theta) * m_laserPose.x + Math.cos(p.theta) * m_laserPose.y;
        lp.theta += m_laserPose.theta;
        int skip = 0;
        double freeDelta = map.getDelta() * m_freeCellRatio;
        for (int rIndex = m_initialBeamsSkip; rIndex < readings.length; rIndex++, angleIndex++) {
            skip++;
            skip = skip > m_likelihoodSkip ? 0 : skip;
            if (skip != 0 || readings[rIndex] > m_usableRange || readings[rIndex] == 0.0) continue;
            DoublePoint phit = new DoublePoint(lp.x, lp.y);
            phit.x += readings[rIndex] * Math.cos(lp.theta + m_laserAngles[angleIndex]);
            phit.y += readings[rIndex] * Math.sin(lp.theta + m_laserAngles[angleIndex]);
            IntPoint iphit = map.world2map(phit);
            DoublePoint pfree = new DoublePoint(lp.x, lp.y);
            pfree.x += (readings[rIndex] - map.getDelta() * freeDelta) * Math.cos(lp.theta + m_laserAngles[angleIndex]);
            pfree.y += (readings[rIndex] - map.getDelta() * freeDelta) * Math.sin(lp.theta + m_laserAngles[angleIndex]);
            pfree.x = pfree.x - phit.x;
            pfree.y = pfree.y - phit.y;

            IntPoint ipfree = map.world2map(pfree);
            boolean found = false;
            DoublePoint bestMu = new DoublePoint(0., 0.);
            for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++) {
                for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++) {
                    IntPoint pr = new IntPoint(iphit.x + xx, iphit.y + yy);
                    IntPoint pf = new IntPoint(pr.x + ipfree.x, pr.y + ipfree.y);
                    int ss = map.getStorage().cellState(pr);
                    if ((ss) > 0) {
                        PointAccumulator cell = (PointAccumulator) map.cell(pr, true);
                        PointAccumulator fcell = (PointAccumulator) map.cell(pf, true);
                        if (cell.doubleValue() > m_fullnessThreshold && fcell.doubleValue() < m_fullnessThreshold) {
                            DoublePoint mu = DoublePoint.minus(phit, cell.mean());
                            if (!found) {
                                bestMu = mu;
                                found = true;
                            } else {
                                bestMu = DoublePoint.mulD(mu, mu) < DoublePoint.mulD(bestMu, bestMu) ? mu : bestMu;
                            }
                        }
                    }
                }
            }
            if (found) {
                s += Math.exp(-1. / m_gaussianSigma * DoublePoint.mulD(bestMu, bestMu));
            }
        }
        return s;
    }

    private class ScoredMove {
        DoubleOrientedPoint pose;
        double score;
        double likelihood;

        private ScoredMove() {
        }

        private ScoredMove(DoubleOrientedPoint pose, double score, double likelihood) {
            this.pose = pose;
            this.score = score;
            this.likelihood = likelihood;
        }
    }
}