package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.sensor.RangeSensor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.core.utils.DoublePoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SharedMemoryGridSlamProcessor extends AbstractGridSlamProcessor {
    private static Logger LOG = LoggerFactory.getLogger(SharedMemoryGridSlamProcessor.class);

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
            GMap lmap = new GMap(new DoublePoint((xmin + xmax) * .5, (ymin + ymax) * .5), xmax - xmin, ymax - ymin, delta);
            Particle p = new Particle(lmap);

            p.pose = new DoubleOrientedPoint(initialPose);
            p.previousPose = initialPose;
            p.setWeight(0);
            p.previousIndex = 0;
            particles.add(p);
            // we use the root directly
            p.node = new TNode(initialPose, 0, null, 0);
        }

        neff = (double) size;
        count = 0;
        readingCount = 0;
        linearDistance = angularDistance = 0;
    }

    public boolean processScan(RangeReading reading, int adaptParticles) {
        DoubleOrientedPoint relPose = reading.getPose();
        if (count == 0) {
            lastPartPose = odoPose = relPose;
        }

        for (Particle p : particles) {
            p.pose = motionModel.drawFromMotion(p.pose, relPose, odoPose);
            // p.pose = relPose;
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

    @Override
    public void scanMatch(double[] plainReading) {

    }

    @Override
    public void setup() {

    }
}
