package cgl.iotrobots.slam.core.sample;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.gridfastsalm.GridSlamProcessor;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.gridfastsalm.TNode;
import cgl.iotrobots.slam.core.scanmatcher.PointAccumulator;
import cgl.iotrobots.slam.core.scanmatcher.ScanMatcher;
import cgl.iotrobots.slam.core.sensor.OdometrySensor;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.sensor.RangeSensor;
import cgl.iotrobots.slam.core.sensor.Sensor;
import cgl.iotrobots.slam.core.utils.OrientedPoint;
import cgl.iotrobots.slam.core.utils.Point;
import cgl.iotrobots.slam.core.utils.Stat;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class Sample {
    private static Logger LOG = LoggerFactory.getLogger(Sample.class);

    GridSlamProcessor gsp_;
    RangeSensor gsp_laser_;
    OdometrySensor gsp_odom_;

    double gsp_laser_angle_increment_;
    double angle_min_;
    double angle_max_;
    int gsp_laser_beam_count_;

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

    String base_frame_;
    String laser_frame_;
    String map_frame_;
    String odom_frame_;

    public static void main(String[] args) {
        Sample sample = new Sample();
        sample.init();

        LaserScan scanI = new LaserScan();
        scanI.angle_increment = Math.PI / 10;
        scanI.angle_max = Math.PI ;
        scanI.angle_min = 0;
        scanI.ranges = new ArrayList<Double>();
        scanI.ranges.add(10.0);
        scanI.ranges.add(10.0);
        scanI.ranges.add(10.0);
        scanI.range_min = 0;
        scanI.range_max = 100;
        sample.initMapper(scanI);

        for (int i = 0; i < 1000; i++) {
            LaserScan scan = new LaserScan();

            scan.angle_increment = Math.PI / 100;
            scan.angle_max = Math.PI ;
            scan.angle_min = 0;
            scan.ranges = new ArrayList<Double>();
            for (int j = 0; j < 1000; j ++) {
                scan.ranges.add(10.0);
            }
            // scan.ranges.add(10.0);
            // scan.ranges.add(10.0);
            scan.range_min = 0;
            scan.range_max = 100;
            sample.laserCallback(scan, null);
        }

        printMap(sample.map_);
    }

    public void init() {
        gsp_ = new GridSlamProcessor();

        throttle_scans_ = 1;

        // gmapping parameters
        maxUrange_ = 0.0;
        maxRange_ = 0.0;
        minimum_score_ = 0;
        sigma_ = 0.05;
        kernelSize_ = 1;
        lstep_ = 0.05;
        astep_ = 0.05;
        iterations_ = 5;
        lsigma_ = 0.075;
        ogain_ = 3.0;
        lskip_ = 0;
        srr_ = 0.1;
        srt_ = 0.2;
        str_ = 0.1;
        stt_ = 0.2;
        linearUpdate_ = 1.0;
        angularUpdate_ = 0.5;
        temporalUpdate_ = 1;
        resampleThreshold_ = 0.5;
        particles_ = 30;
        xmin_ = -100.0;
        ymin_ = -100.0;
        xmax_ = 100.0;
        ymax_ = 100.0;
        delta_ = 0.05;
        occ_thresh_ = 0.25;
        llsamplerange_ = 0.01;
        llsamplestep_ = 0.01;
        lasamplerange_ = 0.005;
        lasamplestep_ = 0.005;


    }

    public boolean initMapper(LaserScan scan) {
        gsp_laser_beam_count_ = scan.ranges.size();

        int orientationFactor = 1;

        angle_min_ = orientationFactor * scan.angle_min;
        angle_max_ = orientationFactor * scan.angle_max;
        gsp_laser_angle_increment_ = orientationFactor * scan.angle_increment;
        LOG.debug("Laser angles top down in laser-frame: min: %.3f max: %.3f inc: %.3f", angle_min_, angle_max_, gsp_laser_angle_increment_);

        OrientedPoint<Double> gmap_pose = new OrientedPoint<Double>(0.0, 0.0, 0.0);

        // setting maxRange and maxUrange here so we can set a reasonable default
        maxRange_ = scan.range_max - 0.01;
        maxUrange_ = maxRange_;

        // The laser must be called "FLASER".
        // We pass in the absolute value of the computed angle increment, on the
        // assumption that GMapping requires a positive angle increment.  If the
        // actual increment is negative, we'll swap the order of ranges before
        // feeding each scan to GMapping.
        gsp_laser_ = new RangeSensor("ROBOTLASER1",
                gsp_laser_beam_count_,
                Math.abs(gsp_laser_angle_increment_),
                gmap_pose,
                0.0,
                maxRange_);

        Map<String, Sensor> smap = new HashMap<String, Sensor>();
        smap.put(gsp_laser_.getName(), gsp_laser_);
        gsp_.setSensorMap(smap);

        gsp_odom_ = new OdometrySensor(odom_frame_, false);

        /// @todo Expose setting an initial pose
        OrientedPoint<Double> initialPose = new OrientedPoint<Double>(0.0, 0.0, 0.0);
        if (getOdomPose(initialPose, scan.timestamp) != null) {
            LOG.warn("Unable to determine inital pose of laser! Starting point will be set to zero.");
            initialPose = new OrientedPoint<Double>(0.0, 0.0, 0.0);
        }

        gsp_.setMatchingParameters(maxUrange_, maxRange_, sigma_,
                kernelSize_, lstep_, astep_, iterations_,
                lsigma_, ogain_, lskip_);

        gsp_.setMotionModelParameters(srr_, srt_, str_, stt_);
        gsp_.setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
        gsp_.setUpdatePeriod_(temporalUpdate_);
        gsp_.getMatcher().setgenerateMap(false);
        gsp_.init(particles_, xmin_, ymin_, xmax_, ymax_, delta_, initialPose);
        gsp_.getMatcher().setllsamplerange(llsamplerange_);
        gsp_.getMatcher().setllsamplestep(llsamplestep_);
        /// @todo Check these calls; in the gmapping gui, they use
        /// llsamplestep and llsamplerange intead of lasamplestep and
        /// lasamplerange.  It was probably a typo, but who knows.
        gsp_.getMatcher().setlasamplerange(lasamplerange_);
        gsp_.getMatcher().setlasamplestep(lasamplestep_);
        gsp_.setMinimumScore(minimum_score_);

        // Call the sampling function once to set the seed.
        Stat.sampleGaussian(1, System.currentTimeMillis());

        LOG.info("Initialization complete");

        return true;
    }

    public void laserCallback(LaserScan scan, OrientedPoint<Double> pose) {
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


        OrientedPoint<Double> odom_pose = new OrientedPoint<Double>(0.0, 0.0, 0.0);
        if (pose != null) {
            odom_pose = pose;
        }
        if (addScan(scan, odom_pose)) {
            LOG.debug("scan processed");

            OrientedPoint<Double> mpose = gsp_.getParticles().get(gsp_.getBestParticleIndex()).pose;
            LOG.debug("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
            LOG.debug("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
            LOG.debug("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

            if (!got_map_ || (scan.timestamp - last_map_update) > map_update_interval_) {
                updateMap(scan);
                last_map_update = scan.timestamp;
                LOG.debug("Updated the map");
            } else {
                updateMap(scan);
            }
        }
        System.out.println("Time: " + (System.currentTimeMillis() - t0) / 1000);
    }

    public boolean addScan(LaserScan scan, OrientedPoint<Double> gmap_pose) {
        if (getOdomPose(gmap_pose, scan.timestamp) == null)
            return false;

        if (scan.ranges.size() != gsp_laser_beam_count_)
            return false;

        // GMapping wants an array of doubles...
        Double[] ranges_double = new Double[scan.ranges.size()];
        // If the angle increment is negative, we have to invert the order of the readings.
        if (gsp_laser_angle_increment_ < 0) {
            LOG.debug("Inverting scan");
            int num_ranges = scan.ranges.size();
            for (int i = 0; i < num_ranges; i++) {
                // Must filter out short readings, because the mapper won't
                if (scan.ranges.get(i) < scan.range_min) {
                    ranges_double[i] = scan.range_max;
                } else {
                    ranges_double[i] = scan.ranges.get(num_ranges - i - 1);
                }
            }
        } else {
            for (int i = 0; i < scan.ranges.size(); i++) {
                // Must filter out short readings, because the mapper won't
                if (scan.ranges.get(i) < scan.range_min) {
                    ranges_double[i] = scan.range_max;
                } else {
                    ranges_double[i] = scan.ranges.get(i);
                }
            }
        }

        RangeReading reading = new RangeReading(scan.ranges.size(),
                ranges_double,
                gsp_laser_,
                scan.timestamp / 1000);

        // ...but it deep copies them in RangeReading constructor, so we don't
        // need to keep our array around.

        reading.setPose(gmap_pose);

        return gsp_.processScan(reading, 0);
    }

    public OutMap map_ = new OutMap();

    private OrientedPoint<Double> getOdomPose(OrientedPoint<Double> gmap_pose, long timestamp) {
        return gmap_pose;
    }

    public void updateMap(LaserScan scan) {
        ScanMatcher matcher = new ScanMatcher();
        double[] laser_angles = new double[scan.ranges.size()];
        double theta = angle_min_;
        for (int i = 0; i < scan.ranges.size(); i++) {
            if (gsp_laser_angle_increment_ < 0)
                laser_angles[scan.ranges.size() - i - 1] = theta;
            else
                laser_angles[i] = theta;
            theta += gsp_laser_angle_increment_;
        }

        matcher.setLaserParameters(scan.ranges.size(), laser_angles,
                gsp_laser_.getPose());

        matcher.setlaserMaxRange(maxRange_);
        matcher.setusableRange(maxUrange_);
        matcher.setgenerateMap(true);

        Particle best =
                gsp_.getParticles().get(gsp_.getBestParticleIndex());

        if (!got_map_) {
            map_.resolution = delta_;
            map_.origin.x = 0.0;
            map_.origin.y = 0.0;
            map_.origin.z = 0.0;
            map_.originOrientation.x = 0.0;
            map_.originOrientation.y = 0.0;
            map_.originOrientation.z = 0.0;
            map_.originOrientation.w = 1.0;
        }

        Point<Double> center = new Point<Double>((xmin_ + xmax_) / 2.0, (ymin_ + ymax_) / 2.0);
        GMap smap = new GMap(center, xmin_, ymin_, xmax_, ymax_, delta_);

        LOG.debug("Trajectory tree:");
        for (TNode n = best.node; n != null; n = n.parent) {
            LOG.debug("{} {} {}", n.pose.x, n.pose.y, n.pose.theta);
            if (n.reading == null) {
                LOG.debug("Reading is NULL");
                continue;
            }
            matcher.invalidateActiveArea();
            double []readingArray = new double[n.reading.size()];
            for (int i = 0 ;i < n.reading.size(); i++) {
                readingArray[i] = n.reading.get(i);
            }
            matcher.computeActiveArea(smap, n.pose, readingArray);
            matcher.registerScan(smap, n.pose, readingArray);
        }

        // the map may have expanded, so resize ros message as well
        if (map_.width != smap.getMapSizeX() || map_.height != smap.getMapSizeY()) {

            // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
            //       so we must obtain the bounding box in a different way
            Point<Double> wmin = smap.map2world(new Point<Integer>(0, 0));
            Point<Double> wmax = smap.map2world(new Point<Integer>(smap.getMapSizeX(), smap.getMapSizeY()));
            xmin_ = wmin.x;
            ymin_ = wmin.y;
            xmax_ = wmax.x;
            ymax_ = wmax.y;

            LOG.debug("map size is now {} x {} pixels ({},{})-({}, {})", smap.getMapSizeX(), smap.getMapSizeY(),
                    xmin_, ymin_, xmax_, ymax_);

            map_.width = smap.getMapSizeX();
            map_.height = smap.getMapSizeY();
            map_.origin.x = xmin_;
            map_.origin.y = ymin_;
            map_.resize(map_.width * map_.height);

            LOG.debug("map origin: (%f, %f)", map_.origin.x, map_.origin.y);
        }
        int count = 0;
        for (int x = 0; x < smap.getMapSizeX(); x++) {
            for (int y = 0; y < smap.getMapSizeY(); y++) {
                /// @todo Sort out the unknown vs. free vs. obstacle thresholding
                Point<Integer> p = new Point<Integer>(x, y);
                PointAccumulator pointAccumulator = (PointAccumulator) smap.cell(p, false);
                double occ = pointAccumulator.doubleValue();
                assert (occ <= 1.0);
                //System.out.println("threshold: "  + occ_thresh_  + " occ: "  + occ);
                if (occ < 0) {
                    map_.data[MAP_IDX(map_.width, x, y)] = -1;
                } else if (occ > occ_thresh_) {
                    //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
                    map_.data[MAP_IDX(map_.width, x, y)] = 100;
                    count++;
                } else {
                    map_.data[MAP_IDX(map_.width, x, y)] = 0;
                }
            }
        }
        System.out.println("count " + count);
        got_map_ = true;
    }

    public static int MAP_IDX(int sx, int i, int j) {
        return ((sx) * (j) + (i));
    }

    public static void printMap(OutMap map) {
        for (int x = 0; x < map.width; x++) {
            for (int y = 0; y < map.height; y++) {
                int occ  = map.data[MAP_IDX(map.width, x, y)];
                if (occ == -1) {
//                    System.out.print("o");
                } else if (occ == 100) {
                    //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
//                    System.out.print("x");
                } else {
                    //System.out.println("v");
                }

            }
            //System.out.print("\n");
        }
    }
}
