package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.particlefilter.UniformResampler;
import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.sensor.*;
import cgl.iotrobots.slam.core.utils.OrientedPoint;
import cgl.iotrobots.slam.core.utils.Point;
import com.google.common.collect.ArrayListMultimap;
import com.google.common.collect.LinkedListMultimap;
import com.google.common.collect.Multimap;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.OutputStream;
import java.io.PrintWriter;
import java.util.*;
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
        LOG.info(" -maxUrange " + urange
                + " -maxUrange " + range
                + " -sigma     " + sigma
                + " -kernelSize " + kernsize
                + " -lstep " + lopt
                + " -lobsGain " + m_obsSigmaGain
                + " -astep " + aopt);
    }

    public void setMotionModelParameters
            (double srr, double srt, double str, double stt) {
        m_motionModel.srr = srr;
        m_motionModel.srt = srt;
        m_motionModel.str = str;
        m_motionModel.stt = stt;
        LOG.info(" -srr " + srr + " -srt " + srt + " -str " + str + " -stt " + stt);
    }

    public void setUpdateDistances(double linear, double angular, double resampleThreshold) {
        m_linearThresholdDistance = linear;
        m_angularThresholdDistance = angular;
        m_resampleThreshold = resampleThreshold;
        LOG.info(" -linearUpdate " + linear
                + " -angularUpdate "+ angular
                + " -resampleThreshold " + m_resampleThreshold );
    }

    public void setSensorMap(Map<String, Sensor> smap){
    /*
      Construct the angle table for the sensor

      FIXME For now detect the readings of only the front laser, and assume its pose is in the center of the robot
    */
        RangeSensor rangeSensor= (RangeSensor) smap.get("ROBOTLASER1");
        m_beams = rangeSensor.beams().size();
        double []angles=new double[rangeSensor.beams().size()];
        for (int i=0; i<m_beams; i++){
            angles[i]=rangeSensor.beams().get(i).pose.theta;
        }
        m_matcher.setLaserParameters(m_beams, angles, rangeSensor.getPose());
    }

    public void init(int size, double xmin, double ymin, double xmax, double ymax, double delta, OrientedPoint<Double> initialPose) {
        m_xmin = xmin;
        m_ymin = ymin;
        m_xmax = xmax;
        m_ymax = ymax;
        m_delta = delta;

        LOG.info(" -xmin " + m_xmin + " -xmax " + m_xmax + " -ymin " + m_ymin
                + " -ymax " + m_ymax + " -delta " + m_delta + " -particles " + size);

        m_particles.clear();

        TNode node = new TNode(initialPose, 0, null, 0);
        GMap lmap = new GMap(new Point<Double>((xmin + xmax) * .5, (ymin + ymax) * .5), xmax - xmin, ymax - ymin, delta);
        for (int i = 0; i < size; i++) {
            int lastIndex = m_particles.size() - 1;
            Particle p = new Particle(lmap);

            p.pose = initialPose;
            p.previousPose = initialPose;
            p.setWeight(0);
            p.previousIndex = 0;
            m_particles.add(p);
            // this is not needed
            //		m_particles.back().node=new TNode(initialPose, 0, node, 0);
            // we use the root directly
            p.node = node;
        }


        m_neff = (double) size;
        m_count = 0;
        m_readingCount = 0;
        m_linearDistance = m_angularDistance = 0;
    }

    public void processTruePos(OdometryReading o) {
        OdometrySensor os = (OdometrySensor) o.getSensor();
        if (os != null && os.isIdeal() && m_outputStream != null) {
            LOG.info("SIMULATOR_POS x:" +  o.getPose().x + " y:" + o.getPose().y + " theta: " + o.getPose().theta);
        }
    }

    public boolean processScan(RangeReading reading, int adaptParticles) {
        OrientedPoint<Double> relPose = reading.getPose();
        if (m_count == 0) {
            m_lastPartPose = m_odoPose = relPose;
        }

        for (Particle p : m_particles) {
            p.pose = m_motionModel.drawFromMotion(p.pose, relPose, m_odoPose);
        }

        LOG.info("ODOM " + m_odoPose.x + " " + m_odoPose.y + " " + m_odoPose.theta + " " + reading.getTime());
        LOG.info("ODO_UPDATE " + m_particles.size() + " ");
        for (Particle p : m_particles) {
            LOG.info("Particle x {}, y {}, theta {}, weight {}", p.pose.x, p.pose.y, p.weight);
        }
        LOG.info("ODO_UPDATE Time {}", reading.getTime());

        //TODO invoke the callback
        onOdometryUpdate();

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

            LOG.info("FRAME " + m_readingCount + " " + m_linearDistance + " " + m_angularDistance);

            LOG.info("update frame " + m_readingCount + "update ld=" + m_linearDistance + " ad=" + m_angularDistance);
            LOG.info("Laser Pose= " + reading.getPose().x + " " + reading.getPose().y + " " + reading.getPose().theta);

            //this is for converting the reading in a scan-matcher feedable form
            assert (reading.size() == m_beams);
            double[] plainReading = new double[m_beams];
            for (int i = 0; i < m_beams; i++) {
                plainReading[i] = reading.get(i);
            }
            LOG.info("m_count " + m_count);

            RangeReading reading_copy =
                    new RangeReading(reading.size(), reading.get(0),
                            (RangeSensor) reading.getSensor(),
                            reading.getTime());

            if (m_count > 0) {
                scanMatch(plainReading);

                LOG.debug("LASER_READING " + reading.size() + " ");
                for (Double b : reading) {
                    LOG.debug(b + " ");
                }
                OrientedPoint p = reading.getPose();

                LOG.debug(p.x + " " + p.y + " " + p.theta + " " + reading.getTime());
                LOG.debug("SM_UPDATE " + m_particles.size() + " ");
                for (Particle it : m_particles) {
                    OrientedPoint pose = it.pose;
                    LOG.debug(pose.x + " " + pose.y + " ");
                    LOG.debug(pose.theta + " " + it.weight + " ");
                }
                onScanmatchUpdate();

                updateTreeWeights(false);

                LOG.info("neff = " + m_neff);
                resample(plainReading, adaptParticles, reading_copy);
            } else {
                LOG.info("Registering First Scan");
                for (Particle it : m_particles) {
                    m_matcher.invalidateActiveArea();
                    m_matcher.computeActiveArea(it.map, it.pose, plainReading);
                    m_matcher.registerScan(it.map, it.pose, plainReading);

                    // cyr: not needed anymore, particles refer to the root in the beginning!
                    TNode node = new TNode(it.pose, 0., it.node, 0);
                    //node->reading=0;
                    node.reading = reading_copy;
                    it.node = node;

                }
            }
            updateTreeWeights(false);

            m_lastPartPose = m_odoPose; //update the past pose for the next iteration
            m_linearDistance = 0;
            m_angularDistance = 0;
            m_count++;
            processed = true;

            //keep ready for the next step
            for (Particle it : m_particles) {
                it.previousPose = it.pose;
            }

        }
        m_readingCount++;
        return processed;
    }

    void updateTreeWeights(boolean weightsAlreadyNormalized) {

        if (!weightsAlreadyNormalized) {
            normalize();
        }
        resetTree();
        propagateWeights();
    }

    void normalize() {
        //normalize the log m_weights
        double gain = 1. / (m_obsSigmaGain * m_particles.size());
        double lmax = Double.MAX_VALUE;
        for (Particle it : m_particles) {
            lmax = it.weight > lmax ? it.weight : lmax;
        }

        m_weights.clear();
        double wcum = 0;
        m_neff = 0;
        for (Particle it : m_particles) {
            double w = Math.exp(gain * (it.weight - lmax));
            m_weights.add(w);
            wcum += w;
        }

        m_neff = 0;
        for (Double it : m_weights) {
            it = it / wcum;
            double w = it;
            m_neff += w * w;
        }
        m_neff = 1. / m_neff;
    }

    public boolean resample(double[] plainReading, int adaptSize, RangeReading reading) {
        boolean hasResampled = false;
        List<TNode> oldGeneration = new ArrayList<TNode>();
        for (Particle m_particle : m_particles) {
            oldGeneration.add(m_particle.node);
        }

        if (m_neff < m_resampleThreshold * m_particles.size()) {
            LOG.info("*************RESAMPLE***************");

            UniformResampler resampler = new UniformResampler();
            m_indexes = resampler.resampleIndexes(m_weights, adaptSize);


            StringBuilder m_outputStream = new StringBuilder("RESAMPLE ").append(m_indexes.size());
            for (Integer it : m_indexes) {
                m_outputStream.append(it).append(" ");
            }
            LOG.debug(m_outputStream.toString());


            onResampleUpdate();
            //BEGIN: BUILDING TREE
            List<Particle> temp = new ArrayList<Particle>();
            int j = 0;
            List<Integer> deletedParticles = new ArrayList<Integer>();        //this is for deleteing the particles which have been resampled away.

            //	 	cerr << "Existing Nodes:" ;
            for (int i = 0; i < m_indexes.size(); i++) {
                //			cerr << " " << m_indexes[i];
                while (j < m_indexes.get(i)) {
                    deletedParticles.add(j);
                    j++;
                }
                if (j == m_indexes.get(i))
                    j++;
                Particle p = m_particles.get(m_indexes.get(i));
                TNode node = null;
                TNode oldNode = oldGeneration.get(m_indexes.get(i));
                //			cerr << i << "->" << m_indexes[i] << "B("<<oldNode->childs <<") ";
                node = new TNode(p.pose, 0, oldNode, 0);
                //node->reading=0;
                node.reading = reading;
                //			cerr << "A("<<node->parent->childs <<") " <<endl;

                temp.add(p);
                p.node = node;
                p.previousIndex = m_indexes.get(i);
            }
            while (j < m_indexes.size()) {
                deletedParticles.add(j);
                j++;
            }
            //		cerr << endl;
            m_outputStream = new StringBuilder("Deleting Nodes:");
            for (int i = 0; i < deletedParticles.size(); i++) {
                m_outputStream.append(" ").append(deletedParticles.get(i));
                m_particles.get(deletedParticles.get(i)).node = null;
            }
            m_outputStream.append(" Done");
            LOG.debug(m_outputStream.toString());

            //END: BUILDING TREE
            LOG.debug("Deleting old particles...");
            m_particles.clear();
            LOG.debug("Done");
            LOG.debug("Copying Particles and  Registering  scans...");
            for (Particle it : m_particles) {
                it.setWeight(0);
                m_matcher.invalidateActiveArea();
                m_matcher.registerScan(it.map, it.pose, plainReading);
                m_particles.add(it);
            }
            hasResampled = true;
        } else {
            int index = 0;
            LOG.debug("Registering Scans:");
            Iterator<TNode> node_it = oldGeneration.iterator();
            for (Particle it : m_particles) {
                //create a new node in the particle tree and add it to the old tree
                //BEGIN: BUILDING TREE
                TNode node = null;
                node = new TNode(it.pose, 0.0, node_it.next(), 0);

                //node->reading=0;
                node.reading = reading;
                it.node = node;

                //END: BUILDING TREE
                m_matcher.invalidateActiveArea();
                m_matcher.registerScan(it.map, it.pose, plainReading);
                it.previousIndex = index;
                index++;
            }
        }
        return hasResampled;
    }

    private void onResampleUpdate() {

    }

    private void onOdometryUpdate() {

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

        int count = 0;
        for (Particle it : m_particles){
            double weight=m_weights.get(count++);
            aw+=weight;
            TNode n=it.node;
            n.accWeight=weight;
            lastNodeWeight+=propagateWeight(n.parent, n.accWeight);
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
            score = m_matcher.optimize(corrected, it.map, it.pose, plainReading);
            //    it->pose=corrected;
            if (score>m_minimumScore){
                it.pose=corrected;
            } else {
                LOG.info("Scan Matching Failed, using odometry. Likelihood=" + l);
                LOG.info("lp:" + m_lastPartPose.x + " "  + m_lastPartPose.y + " " + m_lastPartPose.theta);
                LOG.info("op:" + m_odoPose.x + " " + m_odoPose.y + " " + m_odoPose.theta);
            }

            m_matcher.likelihoodAndScore(s, l, it.map, it.pose, plainReading);
            sumScore+=score;
            it.weight+=l;
            it.weightSum+=l;

            //set up the selective copy of the active area
            //by detaching the areas that will be updated
            m_matcher.invalidateActiveArea();
            m_matcher.computeActiveArea(it.map, it.pose, plainReading);
        }
        LOG.info("Average Scan Matching Score=" + sumScore / m_particles.size());
    }

    List<TNode> getTrajectories() {
        List<TNode> v = new ArrayList<TNode>();
        Multimap<TNode, TNode> parentCache = ArrayListMultimap.create();
        BlockingDeque<TNode> border;

        for (Particle it : m_particles) {
            TNode node = it.node;
            while (node != null) {
                node.flag = false;
                node = node.parent;
            }
        }

        for (Particle it : m_particles) {
            TNode newnode = new TNode(it.node);

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

        while (!border.isEmpty()){
            TNode node=border.poll();
            if (node == null) {
                continue;
            }

            TNode newnode = new TNode(node);
            node.flag=false;

            //update the parent of all of the referring childs
            Collection<TNode> p = parentCache.get(node);
            double childs=0;
            for (TNode it=p.first; it!=p.second; it++){
                assert(it->second->parent==it->first);
                (it->second)->parent=newnode;
                //cerr + "PS(" + it->first + ", "+ it->second + ")";
                childs++;
            }
            ////cerr + endl;
            parentCache.erase(p.first, p.second);
            //cerr + __PRETTY_FUNCTION__ + ": parentCache.size(POSTERASE)=" + parentCache.size() + endl;
            assert(childs==newnode->childs);

            //unmark the node
            if ( node->parent ){
                parentCache.insert(make_pair(node->parent, newnode));
                if(! node->parent->flag){
                    border.push_back(node->parent);
                    node->parent->flag=true;
                }
            }
            //insert the parent in the cache
        }
        //cerr + __PRETTY_FUNCTION__ + " : checking cloned trajectories" + endl;
        for (unsigned int i=0; i<v.size(); i++){
            TNode* node= v[i];
            while (node){
                //cerr +".";
                node=node->parent;
            }
            //cerr + endl;
        }

        return v;

    }getTrajectories() const{
        TNodeVector v;
        TNodeMultimap parentCache;
        TNodeDeque border;

        for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
            TNode* node=it->node;
            while(node){
                node->flag=false;
                node=node->parent;
            }
        }

        for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
            TNode* newnode=new TNode(* (it->node) );

            v.push_back(newnode);
            assert(newnode->childs==0);
            if (newnode->parent){
                parentCache.insert(make_pair(newnode->parent, newnode));
                //cerr + __PRETTY_FUNCTION__ + ": node " + newnode->parent + " flag=" + newnode->parent->flag+ endl;
                if (! newnode->parent->flag){
                    //cerr + __PRETTY_FUNCTION__ + ": node " + newnode->parent + " flag=" + newnode->parent->flag+ endl;
                    newnode->parent->flag=true;
                    border.push_back(newnode->parent);
                }
            }
        }

        //cerr + __PRETTY_FUNCTION__ + ": border.size(INITIAL)=" + border.size() + endl;
        //cerr + __PRETTY_FUNCTION__ + ": parentCache.size()=" + parentCache.size() + endl;
        while (! border.empty()){
            //cerr + __PRETTY_FUNCTION__ + ": border.size(PREPROCESS)=" + border.size() + endl;
            //cerr + __PRETTY_FUNCTION__ + ": parentCache.size(PREPROCESS)=" + parentCache.size() + endl;
            const TNode* node=border.front();
            //cerr + __PRETTY_FUNCTION__ + ": node " + node + endl;
            border.pop_front();
            if (! node)
                continue;

            TNode* newnode=new TNode(*node);
            node->flag=false;

            //update the parent of all of the referring childs
            pair<TNodeMultimap::iterator, TNodeMultimap::iterator> p=parentCache.equal_range(node);
            double childs=0;
            for (TNodeMultimap::iterator it=p.first; it!=p.second; it++){
                assert(it->second->parent==it->first);
                (it->second)->parent=newnode;
                //cerr + "PS(" + it->first + ", "+ it->second + ")";
                childs++;
            }
            ////cerr + endl;
            parentCache.erase(p.first, p.second);
            //cerr + __PRETTY_FUNCTION__ + ": parentCache.size(POSTERASE)=" + parentCache.size() + endl;
            assert(childs==newnode->childs);

            //unmark the node
            if ( node->parent ){
                parentCache.insert(make_pair(node->parent, newnode));
                if(! node->parent->flag){
                    border.push_back(node->parent);
                    node->parent->flag=true;
                }
            }
            //insert the parent in the cache
        }
        //cerr + __PRETTY_FUNCTION__ + " : checking cloned trajectories" + endl;
        for (unsigned int i=0; i<v.size(); i++){
            TNode* node= v[i];
            while (node){
                //cerr +".";
                node=node->parent;
            }
            //cerr + endl;
        }

        return v;

    }

    void integrateScanSequence(TNode node){
        //reverse the list
        TNode aux = node;
        TNode reversed= null;
        double count=0;
        while(aux!=null){
            TNode newnode=new TNode(aux);
            newnode.parent=reversed;
            reversed=newnode;
            aux=aux.parent;
            count++;
        }

        //attach the path to each particle and compute the map;

        LOG.info("Restoring State Nodes=" + count);

        aux=reversed;
        boolean first=true;
        double oldWeight=0;
        OrientedPoint<Double> oldPose;
        while (aux!=null){
            if (first){
                oldPose=aux.pose;
                first=false;
                oldWeight=aux.weight;
            }

            OrientedPoint<Double> dp = new OrientedPoint<Double>(aux.pose.y - oldPose.y, aux.pose.y-oldPose.y, aux.pose.theta-oldPose.theta);
            double dw=aux.weight-oldWeight;
            oldPose=aux.pose;


            double []plainReading = new double[m_beams];
            for(int i=0; i<m_beams; i++) {
                plainReading[i] = aux.reading[i];
            }


            for (Particle it : m_particles){
                //compute the position relative to the path;
                double s=Math.sin(oldPose.theta - it.pose.theta),
                        c=Math.cos(oldPose.theta - it.pose.theta);

                it.pose.x+=c*dp.x-s*dp.y;
                it.pose.y+=s*dp.x+c*dp.y;
                it.pose.theta+=dp.theta;
                it.pose.theta=Math.atan2(Math.sin(it.pose.theta), Math.cos(it.pose.theta));

                //register the scan
                m_matcher.invalidateActiveArea();
                m_matcher.computeActiveArea(it->map, it->pose, plainReading);
                it->weight+=dw;
                it->weightSum+=dw;

                // this should not work, since it->weight is not the correct weight!
                //			it->node=new TNode(it->pose, it->weight, it->node);
                it->node=new TNode(it->pose, 0.0, it->node);
                //update the weight
            }

            delete [] plainReading;
            aux=aux->parent;
        }

        //destroy the path
        aux=reversed;
        while (reversed){
            aux=reversed;
            reversed=reversed->parent;
            delete aux;
        }
    }

//END State Save/Restore

//BEGIN

    void  GridSlamProcessor::updateTreeWeights(bool weightsAlreadyNormalized){

        if (!weightsAlreadyNormalized) {
            normalize();
        }
        resetTree();
        propagateWeights();
    }

    void GridSlamProcessor::resetTree(){
        // don't calls this function directly, use updateTreeWeights(..) !

        for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
            TNode* n=it->node;
            while (n){
                n->accWeight=0;
                n->visitCounter=0;
                n=n->parent;
            }
        }
    }
}
