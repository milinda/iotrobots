package cgl.iotrobots.slam.core.app;

import cgl.iotrobots.slam.core.gridfastsalm.AbstractGridSlamProcessor;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.sensor.OdometrySensor;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.sensor.RangeSensor;
import cgl.iotrobots.slam.core.sensor.Sensor;
import cgl.iotrobots.slam.core.utils.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class GFSAlgorithm {
    private static Logger LOG = LoggerFactory.getLogger(GFSAlgorithm.class);

    public AbstractGridSlamProcessor gsp_;
    RangeSensor gsp_laser_;

    double gspLaserAngleIncrement;
    double angle_min_;
    double angle_max_;
    int gspLaserBeamCount;

    int laser_count_;
    int throttle_scans_;

    boolean got_first_scan_;
    long map_update_interval_;
    boolean got_map_;

    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double minimum_score_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;

    double tf_delay_;

    private MapUpdater mapUpdater;

    public void init() {
        throttle_scans_ = 1;

        // gmapping parameters
        maxUrange_ = 100.0;
        maxRange_ = 100.0;
        minimum_score_ = 0;
        sigma_ = 0.005;
        kernelSize_ = 1;
        lstep_ = 0.05;
        astep_ = 0.05;
        iterations_ = 5;
        lsigma_ = .075;
        ogain_ = 3.0;
        lskip_ = 0;
        srr_ = 0.1;
        srt_ = 0.2;
        str_ = 0.1;
        stt_ = 0.2;
        linearUpdate_ = .05;
        angularUpdate_ = 0.05;
        temporalUpdate_ = 0.0;
        resampleThreshold_ = .5;
        particles_ = 50;
        xmin_ = -20.0;
        ymin_ = -20.0;
        xmax_ = 20.0;
        ymax_ = 20.0;
        delta_ = 0.05;
        occ_thresh_ = 0.25;
        llsamplerange_ = 0.01;
        llsamplestep_ = 0.01;
        lasamplerange_ = 0.005;
        lasamplestep_ = 0.005;
    }


    public boolean initMapper(LaserScan scan) {
        mapUpdater = new MapUpdater(maxRange_, maxUrange_, xmin_, ymin_, xmax_, ymax_, delta_, occ_thresh_);

        gspLaserBeamCount = scan.ranges.size();

        int orientationFactor = 1;

        angle_min_ = orientationFactor * scan.angleMin;
        angle_max_ = orientationFactor * scan.angleMax;
        gspLaserAngleIncrement = orientationFactor * scan.angleIncrement;
        LOG.debug("Laser angles top down in laser-frame: min: %.3f max: %.3f inc: %.3f", angle_min_, angle_max_, gspLaserAngleIncrement);

        DoubleOrientedPoint gmap_pose = new DoubleOrientedPoint(0.0, 0.0, 0.0);

        // setting maxRange and maxUrange here so we can set a reasonable default
        maxRange_ = scan.rangeMax - 0.01;
        maxUrange_ = maxRange_;

        // The laser must be called "FLASER".
        // We pass in the absolute value of the computed angle increment, on the
        // assumption that GMapping requires a positive angle increment.  If the
        // actual increment is negative, we'll swap the order of ranges before
        // feeding each scan to GMapping.
        gsp_laser_ = new RangeSensor("ROBOTLASER1",
                gspLaserBeamCount,
                Math.abs(gspLaserAngleIncrement),
                gmap_pose,
                0.0,
                maxRange_);

        Map<String, Sensor> smap = new HashMap<String, Sensor>();
        smap.put(gsp_laser_.getName(), gsp_laser_);
        gsp_.setSensorMap(smap);

        /// @todo Expose setting an initial pose
        DoubleOrientedPoint initialPose = new DoubleOrientedPoint(0.0, 0.0, 0.0);
        if (getOdomPose(initialPose, scan.timestamp) != null) {
            LOG.warn("Unable to determine inital pose of laser! Starting point will be set to zero.");
            initialPose = new DoubleOrientedPoint(0.0, 0.0, 0.0);
        }

        gsp_.setMatchingParameters(maxUrange_, maxRange_, sigma_,
                kernelSize_, lstep_, astep_, iterations_,
                lsigma_, ogain_, lskip_);

        gsp_.setMotionModelParameters(srr_, srt_, str_, stt_);
        gsp_.setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
        gsp_.setUpdatePeriod_(temporalUpdate_);
        gsp_.getMatcher().setgenerateMap(false);
        gsp_.init(particles_, xmin_, ymin_, xmax_, ymax_, delta_, initialPose);
        gsp_.getMatcher().setLLSamplerange(llsamplerange_);
        gsp_.getMatcher().setLLSamplestep(llsamplestep_);
        /// @todo Check these calls; in the gmapping gui, they use
        /// llsamplestep and llsamplerange intead of lasamplestep and
        /// lasamplerange.  It was probably a typo, but who knows.
        gsp_.getMatcher().setLSSamplerange(lasamplerange_);
        gsp_.getMatcher().setLASamplestep(lasamplestep_);
        gsp_.setMinimumScore(minimum_score_);

        // Call the sampling function once to set the seed.
        Stat.sampleGaussian(1, System.currentTimeMillis());

        LOG.info("Initialization complete");

        return true;
    }

    double totalTime = 0;
    double scanTime = 0;
    int count = 0;

    public void laserScan(LaserScan scan) {
        long t0 =  System.currentTimeMillis();
        laser_count_++;
        if ((laser_count_ % throttle_scans_) != 0)
            return;

        long last_map_update = System.currentTimeMillis();

        // We can't initialize the mapper until we've got the first scan
        if (!got_first_scan_) {
            if (!initMapper(scan))
                return;
            got_first_scan_ = true;
        }

        DoubleOrientedPoint pose = scan.getPose();
        DoubleOrientedPoint odomPose = new DoubleOrientedPoint(0.0, 0.0, 0.0);
        if (pose != null) {
            odomPose = pose;
        }

        if (addScan(scan)) {
            scanTime += System.currentTimeMillis() - t0;
            System.out.println("Add Scan Time: " + (System.currentTimeMillis() - t0) );
            LOG.debug("scan processed");

            DoubleOrientedPoint mpose = gsp_.getParticles().get(gsp_.getBestParticleIndex()).pose;
            LOG.debug("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
            LOG.debug("odom pose: %.3f %.3f %.3f", odomPose.x, odomPose.y, odomPose.theta);
            LOG.debug("correction: %.3f %.3f %.3f", mpose.x - odomPose.x, mpose.y - odomPose.y, mpose.theta - odomPose.theta);

            long t1 = System.currentTimeMillis();
            if (!got_map_ || (scan.timestamp - last_map_update) > map_update_interval_) {
                updateMap(scan);
                LOG.debug("Updated the map");
            } else {
                updateMap(scan);
            }
            System.out.println("Map compute Time: " + (System.currentTimeMillis() - t1) );
        }
        totalTime += System.currentTimeMillis() - t0;
        System.out.println("Total Scan Time: " + (System.currentTimeMillis() - t0));
        count++;
        System.out.println("Average scan time: " + scanTime / count);
        System.out.println("Average total time: " + totalTime / count);
    }

    public boolean addScan(LaserScan scan) {
        DoubleOrientedPoint pose = scan.getPose();

        if (getOdomPose(pose, scan.timestamp) == null) {
            return false;
        }

        if (scan.ranges.size() != gspLaserBeamCount) {
            return false;
        }
        Double[] ranges_double = Utils.getDoubles(scan, gspLaserAngleIncrement);
        RangeReading reading = new RangeReading(scan.ranges.size(),
                ranges_double,
                gsp_laser_,
                scan.timestamp);
        reading.setPose(pose);

        return gsp_.processScan(reading, 0);
    }

    public GFSMap getMap() {
        return this.mapUpdater.getMap();
    }

    private DoubleOrientedPoint getOdomPose(DoubleOrientedPoint gmap_pose, long timestamp) {
        return gmap_pose;
    }

    public void updateMap(LaserScan scan) {
        double[] laser_angles = new double[scan.ranges.size()];
        double theta = angle_min_;
        for (int i = 0; i < scan.ranges.size(); i++) {
            if (gspLaserAngleIncrement < 0)
                laser_angles[scan.ranges.size() - i - 1] = theta;
            else
                laser_angles[i] = theta;
            theta += gspLaserAngleIncrement;
        }

//        double angle = Math.PI * 2 -.5 * res * beams_num;
        double angle = -.5 * gspLaserAngleIncrement * gspLaserBeamCount;
        // double angle = 0;
        for (int i = 0; i < gspLaserBeamCount; i++, angle += gspLaserAngleIncrement) {
            laser_angles[i] = angle;
        }

        Particle best =
                gsp_.getParticles().get(gsp_.getBestParticleIndex());
        System.out.format("best particle pose: %f %f %f", best.getPose().getX(), best.getPose().getY(), best.getPose().getTheta());
        mapUpdater.updateMap(best, laser_angles, gsp_laser_.getPose());
    }

    public static int MAP_IDX(int sx, int i, int j) {
        return ((sx) * (j) + (i));
    }
}
