package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.sensor.*;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import com.google.common.collect.ArrayListMultimap;
import com.google.common.collect.Multimap;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

public class GridSlamProcessor extends SharedMemoryGridSlamProcessor {
    private static Logger LOG = LoggerFactory.getLogger(GridSlamProcessor.class);

    public void processTruePos(OdometryReading o) {
        OdometrySensor os = (OdometrySensor) o.getSensor();
        LOG.info("SIMULATOR_POS x:" + o.getPose().x + " y:" + o.getPose().y + " theta: " + o.getPose().theta);
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



    /**
     * Just scan match every single particle.
     * If the scan matching fails, the particle gets a default likelihood.
     */
    public void scanMatch(double[] plainReading) {
        // sample a new pose from each scan in the reference
        double sumScore = 0;
        for (Particle it : particles) {
            sumScore += scanMatchParticle(plainReading, 0, it);
        }
        LOG.info("Average Scan Matching Score = " + sumScore / particles.size());
    }

    @Override
    public void setup() {

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
