package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.particlefilter.UniformResampler;
import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.sensor.*;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.core.utils.DoublePoint;
import com.google.common.collect.ArrayListMultimap;
import com.google.common.collect.Multimap;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

public class GridSlamProcessor extends AbstractGridSlamProcessor {
    private static Logger LOG = LoggerFactory.getLogger(GridSlamProcessor.class);



    public void init(int size, double xmin, double ymin, double xmax, double ymax, double delta, DoubleOrientedPoint initialPose) {
        this.xmin = xmin;
        this.ymin = ymin;
        this.xmax = xmax;
        this.ymax = ymax;
        this.delta = delta;

        LOG.info(" -xmin " + this.xmin + " -xmax " + this.xmax + " -ymin " + this.ymin
                + " -ymax " + this.ymax + " -delta " + this.delta + " -particles " + size);

        particles.clear();

        for (int i = 0; i < size; i++) {
            int lastIndex = particles.size() - 1;
            GMap lmap = new GMap(new DoublePoint((xmin + xmax) * .5, (ymin + ymax) * .5), xmax - xmin, ymax - ymin, delta);
            Particle p = new Particle(lmap);

            p.pose = new DoubleOrientedPoint(initialPose);
            p.previousPose = initialPose;
            p.setWeight(0);
            p.previousIndex = 0;
            particles.add(p);
            // we use the root directly
            TNode node = new TNode(initialPose, 0, null, 0);
            p.node = node;
        }


        neff = (double) size;
        count = 0;
        readingCount = 0;
        linearDistance = angularDistance = 0;
    }

    public void processTruePos(OdometryReading o) {
        OdometrySensor os = (OdometrySensor) o.getSensor();
        LOG.info("SIMULATOR_POS x:" + o.getPose().x + " y:" + o.getPose().y + " theta: " + o.getPose().theta);
    }

    public boolean processScan(RangeReading reading, int adaptParticles) {
        DoubleOrientedPoint relPose = reading.getPose();
        if (count == 0) {
            lastPartPose = odoPose = relPose;
        }

        for (Particle p : particles) {
            p.pose = motionModel.drawFromMotion(p.pose, relPose, odoPose);
            p.pose = relPose;
        }

        LOG.info("ODOM " + odoPose.x + " " + odoPose.y + " " + odoPose.theta + " " + reading.getTime());
        LOG.info("ODO_UPDATE " + particles.size() + " ");
        for (Particle p : particles) {
            LOG.info("Particle x {}, y {}, theta {}, weight {}", p.pose.x, p.pose.y, p.weight);
        }
        LOG.info("ODO_UPDATE Time {}", reading.getTime());

        DoubleOrientedPoint move = DoubleOrientedPoint.minus(relPose, odoPose);
        move.theta = Math.atan2(Math.sin(move.theta), Math.cos(move.theta));
        linearDistance += Math.sqrt(DoubleOrientedPoint.mulN(move, move));
        angularDistance += Math.abs(move.theta);

        // if the robot jumps throw a warning
        if (linearDistance > distanceThresholdCheck) {
            LOG.error("***********************************************************************");
            LOG.error("********** Error: distanceThresholdCheck overridden!!!! *************");
            LOG.error("distanceThresholdCheck=" + distanceThresholdCheck);
            LOG.error("Old Odometry Pose= " + odoPose.x + " " + odoPose.y + " " + odoPose.theta);
            LOG.error("New Odometry Pose (reported from observation)= " + relPose.x + " " + relPose.y + " " + relPose.theta);
            LOG.error("***********************************************************************");
            LOG.error("** The Odometry has a big jump here. This is probably a bug in the   **");
            LOG.error("** odometry/laser input. We continue now, but the result is probably **");
            LOG.error("** crap or can lead to a core dump since the map doesn't fit.... C&G **");
            LOG.error("***********************************************************************");
        }

        odoPose = relPose;
        boolean processed = false;

        // process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
        if (count == 0
                || linearDistance >= linearThresholdDistance
                || angularDistance >= angularThresholdDistance
                || (period_ >= 0.0 && (reading.getTime() - last_update_time_) > period_)) {
            last_update_time_ = reading.getTime();

            LOG.info("FRAME " + readingCount + " " + linearDistance + " " + angularDistance);

            LOG.info("update frame " + readingCount + "update ld=" + linearDistance + " ad=" + angularDistance);
            LOG.info("Laser Pose= " + reading.getPose().x + " " + reading.getPose().y + " " + reading.getPose().theta);

            //this is for converting the reading in a scan-matcher feedable form
            assert (reading.size() == beams);
            double[] plainReading = new double[beams];
            for (int i = 0; i < beams; i++) {
                plainReading[i] = reading.get(i);
            }
            LOG.info("count " + count);

            RangeReading reading_copy =
                    new RangeReading(reading.size(), reading.toArray(new Double[reading.size()]),
                            (RangeSensor) reading.getSensor(),
                            reading.getTime());

            if (count > 0) {
                scanMatch(plainReading);

                LOG.debug("LASER_READING " + reading.size() + " ");
                for (Double b : reading) {
                    LOG.debug(b + " ");
                }
                DoubleOrientedPoint p = reading.getPose();

                LOG.debug(p.x + " " + p.y + " " + p.theta + " " + reading.getTime());
                LOG.debug("SM_UPDATE " + particles.size() + " ");
                for (Particle it : particles) {
                    DoubleOrientedPoint pose = it.pose;
                    LOG.debug(pose.x + " " + pose.y + " ");
                    LOG.debug(pose.theta + " " + it.weight + " ");
                }

                updateTreeWeights(false);

                LOG.info("neff = " + neff);
                resample(plainReading, adaptParticles, reading_copy);
            } else {
                LOG.info("Registering First Scan");
                for (Particle it : particles) {
                    matcher.invalidateActiveArea();
                    matcher.computeActiveArea(it.map, it.pose, plainReading);
                    matcher.registerScan(it.map, it.pose, plainReading);

                    // not needed anymore, particles refer to the root in the beginning!
                    TNode node = new TNode(it.pose, 0., it.node, 0);
                    node.reading = reading_copy;
                    it.node = node;

                }
            }
            updateTreeWeights(false);

            lastPartPose = odoPose; //update the past pose for the next iteration
            linearDistance = 0;
            angularDistance = 0;
            count++;
            processed = true;

            //keep ready for the next step
            for (Particle it : particles) {
                it.previousPose = it.pose;
            }

        }
        readingCount++;
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
        //normalize the log weights
        double gain = 1. / (obsSigmaGain * particles.size());
        double lmax = -Double.MAX_VALUE;
        for (Particle it : particles) {
            lmax = it.weight > lmax ? it.weight : lmax;
        }

        weights.clear();
        double wcum = 0;
        neff = 0;
        for (Particle it : particles) {
            double w = Math.exp(gain * (it.weight - lmax));
            weights.add(w);
            wcum += w;
        }

        neff = 0;
        for (Double it : weights) {
            it = it / wcum;
            double w = it;
            neff += w * w;
        }
        neff = 1. / neff;
    }

    public boolean resample(double[] plainReading, int adaptSize, RangeReading reading) {
        boolean hasResampled = false;
        List<TNode> oldGeneration = new ArrayList<TNode>();
        for (Particle m_particle : particles) {
            oldGeneration.add(m_particle.node);
        }

        if (neff < resampleThreshold * particles.size()) {
            LOG.info("*************RESAMPLE***************");
            UniformResampler resampler = new UniformResampler();
            indexes = resampler.resampleIndexes(weights, adaptSize);

            StringBuilder m_outputStream = new StringBuilder("RESAMPLE ").append(indexes.size());
            for (Integer it : indexes) {
                m_outputStream.append(it).append(" ");
            }
            LOG.debug(m_outputStream.toString());


            //begin building tree
            List<Particle> temp = new ArrayList<Particle>();
            int j = 0;
            //this is for deleteing the particles which have been resampled away.
            List<Integer> deletedParticles = new ArrayList<Integer>();

            for (int i = 0; i < indexes.size(); i++) {
                while (j < indexes.get(i)) {
                    deletedParticles.add(j);
                    j++;
                }
                if (j == indexes.get(i)) {
                    j++;
                }
                Particle p = new Particle(particles.get(indexes.get(i)));

                TNode node ;
                TNode oldNode = oldGeneration.get(indexes.get(i));
                node = new TNode(p.pose, 0, oldNode, 0);
                node.reading = reading;

                temp.add(p);
                p.node = node;
                p.previousIndex = indexes.get(i);
            }
            while (j < indexes.size()) {
                deletedParticles.add(j);
                j++;
            }
            m_outputStream = new StringBuilder("Deleting Nodes:");
            for (int i = 0; i < deletedParticles.size(); i++) {
                m_outputStream.append(" ").append(deletedParticles.get(i));
                particles.get(deletedParticles.get(i)).node = null;
            }
            LOG.debug(m_outputStream.toString());

            LOG.debug("Deleting old particles...");
            particles.clear();
            LOG.debug("Copying Particles and  Registering  scans...");
            for (Particle it : temp) {
                it.setWeight(0);
                matcher.invalidateActiveArea();
                matcher.registerScan(it.map, it.pose, plainReading);
                particles.add(it);
            }
            hasResampled = true;
        } else {
            int index = 0;
            LOG.debug("Registering Scans:");
            Iterator<TNode> node_it = oldGeneration.iterator();
            for (Particle it : particles) {
                //create a new node in the particle tree and add it to the old tree
                TNode node = null;
                node = new TNode(it.pose, 0.0, node_it.next(), 0);

                node.reading = reading;
                it.node = node;

                matcher.invalidateActiveArea();
                matcher.registerScan(it.map, it.pose, plainReading);
                it.previousIndex = index;
                index++;
            }
        }
        return hasResampled;
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
        DoubleOrientedPoint oldPose = new DoubleOrientedPoint(0.0, 0.0, 0.0);
        while (aux != null) {
            if (first) {
                oldPose = aux.pose;
                first = false;
                oldWeight = aux.weight;
            }

            DoubleOrientedPoint dp = DoubleOrientedPoint.minus(aux.pose, oldPose);
            double dw = aux.weight - oldWeight;
            oldPose = aux.pose;


            double[] plainReading = new double[beams];
            for (int i = 0; i < beams; i++) {
                plainReading[i] = aux.reading.get(i);
            }


            for (Particle it : particles) {
                //compute the position relative to the path;
                double s = Math.sin(oldPose.theta - it.pose.theta),
                        c = Math.cos(oldPose.theta - it.pose.theta);

                it.pose.x += c * dp.x - s * dp.y;
                it.pose.y += s * dp.x + c * dp.y;
                it.pose.theta += dp.theta;
                it.pose.theta = Math.atan2(Math.sin(it.pose.theta), Math.cos(it.pose.theta));

                //register the scan
                matcher.invalidateActiveArea();
                matcher.computeActiveArea(it.map, it.pose, plainReading);
                it.weight += dw;
                it.weightSum += dw;

                // this should not work, since it->weight is not the correct weight!
                //			it->node=new TNode(it->pose, it->weight, it->node);
                it.node = new TNode(it.pose, 0.0, it.node, 0);
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

    public void resetTree() {
        // don't calls this function directly, use updateTreeWeights(..) !
        for (Particle it : particles) {
            TNode n = it.node;
            while (n != null) {
                n.accWeight = 0;
                n.visitCounter = 0;
                n = n.parent;
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

    double propagateWeights() {
        // don't calls this function directly, use updateTreeWeights(..) !
        // all nodes must be resetted to zero and weights normalized
        // the accumulated weight of the root
        double lastNodeWeight = 0;
        // sum of the weights in the leafs
        double aw = 0;

        int count = 0;
        for (Particle it : particles) {
            double weight = weights.get(count++);
            aw += weight;
            TNode n = it.node;
            n.accWeight = weight;
            lastNodeWeight += propagateWeight(n.parent, n.accWeight);
        }

        if (Math.abs(aw - 1.0) > 0.0001 || Math.abs(lastNodeWeight - 1.0) > 0.0001) {
            LOG.error("ERROR: root->accWeight=" + lastNodeWeight + "    sum_leaf_weights=" + aw);
        }
        return lastNodeWeight;
    }

    /**
     * Just scan match every single particle.
     * If the scan matching fails, the particle gets a default likelihood.
     */
    public void scanMatch(double[] plainReading) {
        // sample a new pose from each scan in the reference
        double sumScore = 0;
        for (Particle it : particles) {
            DoubleOrientedPoint corrected = new DoubleOrientedPoint(0.0, 0.0, 0.0);
            double score = 0, l, s;
            score = matcher.optimize(corrected, it.map, it.pose, plainReading);
            //    it->pose=corrected;
            if (score > minimumScore) {
                it.pose = new DoubleOrientedPoint(corrected);
            } else {
                LOG.info("Scan Matching Failed, using odometry. Likelihood=");
                LOG.info("lp:" + lastPartPose.x + " " + lastPartPose.y + " " + lastPartPose.theta);
                LOG.info("op:" + odoPose.x + " " + odoPose.y + " " + odoPose.theta);
            }

            ScanMatcher.LikeliHoodAndScore score1 = matcher.likelihoodAndScore(it.map, it.pose, plainReading);
            l = score1.l;
            s = score1.s;
//            LikeliHood likeliHood = matcher.likelihood(it.map, it.pose, plainReading);
//            l = likeliHood.likeliHood;

            sumScore += score;
            it.weight += l;
            it.weightSum += l;

            //set up the selective copy of the active area
            //by detaching the areas that will be updated
            matcher.invalidateActiveArea();
            matcher.computeActiveArea(it.map, it.pose, plainReading);
        }
        LOG.info("Average Scan Matching Score=" + sumScore / particles.size());
    }

    List<TNode> getTrajectories() {
        List<TNode> v = new ArrayList<TNode>();
        Multimap<TNode, TNode> parentCache = ArrayListMultimap.create();
        BlockingDeque<TNode> border = new LinkedBlockingDeque<TNode>();

        for (Particle it : particles) {
            TNode node = it.node;
            while (node != null) {
                node.flag = false;
                node = node.parent;
            }
        }

        for (Particle it : particles) {
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

        while (!border.isEmpty()) {
            TNode node = border.poll();
            if (node == null) {
                continue;
            }

            TNode newnode = new TNode(node);
            node.flag = false;

            //update the parent of all of the referring childs
            Collection<TNode> p = parentCache.get(node);
            double childs = 0;
            for (TNode it : p) {
                assert (it.parent == node);
                it.parent = newnode;
                childs++;
            }
            parentCache.removeAll(node);
            assert (childs == newnode.childs);

            //unmark the node
            if (node.parent != null) {
                parentCache.put(node.parent, newnode);
                if (!node.parent.flag) {
                    try {
                        border.putLast(node.parent);
                    } catch (InterruptedException e) {
                        LOG.error("Failed to push", e);
                    }
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
}
