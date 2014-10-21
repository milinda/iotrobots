package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.sensor.OdometryReading;
import cgl.iotrobots.slam.core.sensor.OdometrySensor;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.utils.OrientedPoint;
import cgl.iotrobots.slam.core.utils.Point;
import com.google.common.collect.ArrayListMultimap;
import com.google.common.collect.LinkedListMultimap;
import com.google.common.collect.Multimap;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.OutputStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

public class GridSlamProcessor {
    private static Logger LOG = LoggerFactory.getLogger(GridSlamProcessor.class);

    private static final double m_distanceThresholdCheck = 20;

    private List<TNode> TNodeVector = new ArrayList<TNode>();
    private BlockingDeque<TNode> TNodeDeque = new LinkedBlockingDeque<TNode>();
    private List<Particle> m_particles = new ArrayList<Particle>();

    List<Integer> m_indexes =  new ArrayList<Integer>();
    List<Double> m_weights = new ArrayList<Double>();

    ScanMatcher m_matcher;

    MotionModel m_motionModel;

    int m_beams;
    double last_update_time_;
    double period_;

    double m_minimumScore;

    double m_resampleThreshold;

    int m_count, m_readingCount;
    OrientedPoint<Double> m_lastPartPose;
    OrientedPoint<Double> m_odoPose;
    OrientedPoint<Double> m_pose;

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

    PrintWriter m_outputStream;

    public GridSlamProcessor() {
        period_ = 5.0;
        m_obsSigmaGain = 1;
        m_resampleThreshold = 0.5;
        m_minimumScore = 0.;
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

    public void init(int size, double xmin, double ymin, double xmax, double ymax, double delta, OrientedPoint initialPose) {
        m_xmin = xmin;
        m_ymin = ymin;
        m_xmax = xmax;
        m_ymax = ymax;
        m_delta = delta;

        m_particles.clear();

        TNode node = new TNode(initialPose, 0, null, 0);
        GMap lmap = new GMap(new Point<Double>((xmin + xmax) * .5, (ymin + ymax) * .5), xmax - xmin, ymax - ymin, delta);
        for (int i = 0; i < size; i++) {
            int lastIndex = m_particles.size() - 1;
            m_particles.add(new Particle(lmap));
            m_particles.get(lastIndex).pose = initialPose;
            m_particles.get(lastIndex).previousPose = initialPose;
            m_particles.get(lastIndex).setWeight(0);
            m_particles.get(lastIndex).previousIndex = 0;
            // this is not needed
            //		m_particles.back().node=new TNode(initialPose, 0, node, 0);
            // we use the root directly
            m_particles.get(lastIndex).node = node;
        }


        m_neff = (double) size;
        m_count = 0;
        m_readingCount = 0;
        m_linearDistance = m_angularDistance = 0;
    }

    public void processTruePos(OdometryReading o) {
        OdometrySensor os = (OdometrySensor) o.getSensor();
        if (os != null && os.isIdeal() && m_outputStream != null) {
            // TODO:write something
        }
    }

    public void processScan(RangeReading reading, int adaptParticles) {
        OrientedPoint<Double> relPose = reading.getPose();
        if (m_count == 0) {
            m_lastPartPose = m_odoPose = relPose;
        }

        for (Particle p : m_particles) {
            p.pose = m_motionModel.drawFromMotion(p.pose, relPose, m_odoPose);
        }

        // todo outto to file

        //TODO invoke the callback
        // onOdometryUpdate();

        OrientedPoint<Double> move = OrientedPoint.minus(relPose, m_odoPose);
        move.theta = Math.atan2(Math.sin(move.theta), Math.cos(move.theta));
        m_linearDistance += Math.sqrt(OrientedPoint.mulN(move, move));
        m_angularDistance += Math.abs(move.theta);

        // if the robot jumps throw a warning
        if (m_linearDistance > m_distanceThresholdCheck) {
            LOG.error("***********************************************************************");
            LOG.error("********** Error: m_distanceThresholdCheck overridden!!!! *************");
            LOG.error("m_distanceThresholdCheck=" + m_distanceThresholdCheck);
            LOG.error("Old Odometry Pose= " + m_odoPose.x + " " + m_odoPose.y + " " + m_odoPose.theta);
            LOG.error("New Odometry Pose (reported from observation)= " + relPose.x + " " + relPose.y + " " + relPose.theta);
            LOG.error("***********************************************************************");
            LOG.error("** The Odometry has a big jump here. This is probably a bug in the   **");
            LOG.error("** odometry/laser input. We continue now, but the result is probably **");
            LOG.error("** crap or can lead to a core dump since the map doesn't fit.... C&G **");
            LOG.error("***********************************************************************");
        }

        m_odoPose = relPose;
        boolean processed = false;

        // process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
        if (m_count == 0
                || m_linearDistance >= m_linearThresholdDistance
                || m_angularDistance >= m_angularThresholdDistance
                || (period_ >= 0.0 && (reading.getTime() - last_update_time_) > period_)) {
            last_update_time_ = reading.getTime();

            if (m_outputStream != null) {
                m_outputStream.write(setiosflags(ios::fixed) << setprecision(6);
                m_outputStream << "FRAME " << m_readingCount;
                m_outputStream << " " << m_linearDistance;
                m_outputStream << " " << m_angularDistance << endl;
            }

            LOG.info("update frame " + m_readingCount + "update ld=" + m_linearDistance + " ad=" + m_angularDistance);
            LOG.info("Laser Pose= " + reading.getPose().x + " " + reading.getPose().y + " " + reading.getPose().theta);

            //this is for converting the reading in a scan-matcher feedable form
            assert (reading.size() == m_beams);
            double []plainReading = new double[m_beams];
            for (int i = 0; i<m_beams; i++){
                plainReading[i] = reading.get(i);
            }
            LOG.info("m_count " + m_count);

            RangeReading reading_copy =
                    new RangeReading(reading.size(), & (reading[0]),
                    static_cast <const RangeSensor * > (reading.getSensor()),
                    reading.getTime());

            if (m_count > 0) {
                scanMatch(plainReading);
                if (m_outputStream.is_open()) {
                    m_outputStream << "LASER_READING " << reading.size() << " ";
                    m_outputStream << setiosflags(ios::fixed) << setprecision(2);
                    for (RangeReading::const_iterator b = reading.begin(); b != reading.end() ;
                    b++){
                        m_outputStream <<*b << " ";
                    }
                    OrientedPoint p = reading.getPose();
                    m_outputStream << setiosflags(ios::fixed) << setprecision(6);
                    m_outputStream << p.x << " " << p.y << " " << p.theta << " " << reading.getTime() << endl;
                    m_outputStream << "SM_UPDATE " << m_particles.size() << " ";
                    for (ParticleVector::const_iterator it = m_particles.begin(); it != m_particles.end() ;
                    it++){
                        const OrientedPoint & pose = it -> pose;
                        m_outputStream << setiosflags(ios::fixed) << setprecision(3) << pose.x << " " << pose.y << " ";
                        m_outputStream << setiosflags(ios::fixed) << setprecision(6) << pose.theta << " " << it -> weight << " ";
                    }
                    m_outputStream << endl;
                }
                onScanmatchUpdate();

                updateTreeWeights(false);

                LOG.info("neff= " + m_neff);
                if (m_outputStream.is_open()) {
                    m_outputStream << setiosflags(ios::fixed) << setprecision(6);
                    m_outputStream << "NEFF " << m_neff << endl;
                }
                resample(plainReading, adaptParticles, reading_copy);
            } else {
                m_infoStream << "Registering First Scan" << endl;
                for (ParticleVector::iterator it = m_particles.begin(); it != m_particles.end() ;
                it++){
                    m_matcher.invalidateActiveArea();
                    m_matcher.computeActiveArea(it -> map, it -> pose, plainReading);
                    m_matcher.registerScan(it -> map, it -> pose, plainReading);

                    // cyr: not needed anymore, particles refer to the root in the beginning!
                    TNode * node = new TNode(it -> pose, 0., it -> node, 0);
                    //node->reading=0;
                    node -> reading = reading_copy;
                    it -> node = node;

                }
            }
            //		cerr  << "Tree: normalizing, resetting and propagating weights at the end..." ;
            updateTreeWeights(false);
            //		cerr  << ".done!" <<endl;

            delete[] plainReading;
            m_lastPartPose = m_odoPose; //update the past pose for the next iteration
            m_linearDistance = 0;
            m_angularDistance = 0;
            m_count++;
            processed = true;

            //keep ready for the next step
            for (ParticleVector::iterator it = m_particles.begin(); it != m_particles.end() ;
            it++){
                it -> previousPose = it -> pose;
            }

        }
        if (m_outputStream.is_open())
            m_outputStream << flush;
        m_readingCount++;
        return processed;
    }

    private void onScanmatchUpdate() {

    }

    public List<TNode> getTrajectories() {
        List<TNode> v = new ArrayList<TNode>();
        Multimap<TNode, TNode> parentCache = ArrayListMultimap.create();
        BlockingDeque<TNode> border = new LinkedBlockingDeque<TNode>();

        for (Particle particle : m_particles) {
            TNode node = particle.node;
            while (node != null) {
                node.flag = false;
                node = node.parent;
            }
        }

        for (Particle particle : m_particles) {
            TNode newnode = new TNode();

            v.add(newnode);
            assert (newnode.childs == 0);
            if (newnode.parent != null) {
                parentCache.put(newnode.parent, newnode);
                if (!newnode.parent.flag) {
                    newnode.parent.flag = true;
                    border.add(newnode.parent);
                }
            }
        }

        while (!border.isEmpty()) {
            TNode node = border.poll();
            if (node != null)
                continue;

            TNode newnode = new TNode(node);
            node.flag = false;

            //update the parent of all of the referring childs
            Collection<TNode> p = parentCache.get(node);
            double childs = 0;
            for (TNode second : p){
                assert (second.parent.equals(node));
                second.parent = newnode;
                childs++;
            }
            parentCache.remove(node, p.second);
            assert (childs == newnode.childs);
            //unmark the node
            if (node.parent != null) {
                parentCache.put(node.parent, newnode);
                if (!node.parent.flag) {
                    border.add(node.parent);
                    node.parent.flag = true;
                }
            }
            //insert the parent in the cache
        }
        for (TNode node : v) {
            while (node != null) {
                node = node.parent;
            }
        }

        return v;

    }

    void integrateScanSequence(TNode node) {
        //reverse the list
        TNode aux = node;
        TNode reversed = null;
        double count = 0;
        while (aux != null) {
            TNode newnode = new TNode(aux);
            newnode.parent = reversed;
            reversed = newnode;
            aux = aux.parent;
            count++;
        }

        //attach the path to each particle and compute the map;
        LOG.info("Restoring State Nodes=" + count);

        aux = reversed;
        boolean first = true;
        double oldWeight = 0;
        OrientedPoint<Double> oldPose;
        while (aux != null) {
            if (first) {
                oldPose = aux.pose;
                first = false;
                oldWeight = aux.weight;
            }

            OrientedPoint<Double> dp = OrientedPoint.minus(aux.pose, oldPose);
            double dw = aux.weight - oldWeight;
            oldPose = aux.pose;


            double[] plainReading = new double[m_beams];
            for (int i = 0; i < m_beams; i++) {
                plainReading[i] = ( * (aux.reading))[i];
            }


            for (Particle it : m_particles) {
                //compute the position relative to the path;
                double s = Math.sin(oldPose.theta - it.pose.theta),
                        c = Math.cos(oldPose.theta - it.pose.theta);

                it.pose.x += c * dp.x - s * dp.y;
                it.pose.y += s * dp.x + c * dp.y;
                it.pose.theta += dp.theta;
                it.pose.theta = Math.atan2(Math.sin(it.pose.theta), Math.cos(it.pose.theta));

                //register the scan
                m_matcher.invalidateActiveArea();
                m_matcher.computeActiveArea(it -> map, it -> pose, plainReading);
                it.weight += dw;
                it.weightSum += dw;

                // this should not work, since it->weight is not the correct weight!
                //			it->node=new TNode(it->pose, it->weight, it->node);
                it.node = new TNode(it.pose, 0.0, it.node);
                //update the weight
            }

            aux = aux.parent;
        }

        //destroy the path
        aux = reversed;
        while (reversed != null) {
            aux = reversed;
            reversed = reversed.parent;
        }
    }

    public void resetTree(){
        // don't calls this function directly, use updateTreeWeights(..) !

        for (Particle it : m_particles){
            TNode n = it.node;
            while (n != null){
                n.accWeight=0;
                n.visitCounter=0;
                n=n.parent;
            }
        }
    }

    double propagateWeight(TNode n, double weight) {
        if (n == null) {
            return weight;
        }
        double w = 0;
        n.visitCounter++;
        n.accWeight += weight;
        if (n.visitCounter == n.childs) {
            w = propagateWeight(n.parent, n.accWeight);
        }
        assert (n.visitCounter <= n.childs);
        return w;
    }

    double propagateWeights(){
        // don't calls this function directly, use updateTreeWeights(..) !

        // all nodes must be resetted to zero and weights normalized

        // the accumulated weight of the root
        double lastNodeWeight=0;
        // sum of the weights in the leafs
        double aw=0;

        std::vector<double>::iterator w=m_weights.begin();
        for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
            double weight=*w;
            aw+=weight;
            TNode * n=it->node;
            n->accWeight=weight;
            lastNodeWeight+=propagateWeight(n->parent,n->accWeight);
            w++;
        }

        if (Math.abs(aw-1.0) > 0.0001 || Math.abs(lastNodeWeight-1.0) > 0.0001) {
            LOG.error("ERROR: root->accWeight=" + lastNodeWeight + "    sum_leaf_weights=" + aw);
        }
        return lastNodeWeight;
    }

    /**Just scan match every single particle.
     If the scan matching fails, the particle gets a default likelihood.*/
    void scanMatch(double []plainReading){
        // sample a new pose from each scan in the reference

        double sumScore=0;
        for (Particle it : m_particles){
            OrientedPoint<Double> corrected;
            double score, l, s;
            score = m_matcher.optimize(corrected, it->map, it->pose, plainReading);
            //    it->pose=corrected;
            if (score>m_minimumScore){
                it->pose=corrected;
            } else {
                LOG.info("Scan Matching Failed, using odometry. Likelihood=" + l);
                LOG.info("lp:" + m_lastPartPose.x + " "  + m_lastPartPose.y + " " + m_lastPartPose.theta);
                LOG.info("op:" + m_odoPose.x + " " + m_odoPose.y + " " + m_odoPose.theta);
            }

            m_matcher.likelihoodAndScore(s, l, it->map, it->pose, plainReading);
            sumScore+=score;
            it->weight+=l;
            it->weightSum+=l;

            //set up the selective copy of the active area
            //by detaching the areas that will be updated
            m_matcher.invalidateActiveArea();
            m_matcher.computeActiveArea(it->map, it->pose, plainReading);
        }
        LOG.info("Average Scan Matching Score=" + sumScore / m_particles.size());
    }
}
