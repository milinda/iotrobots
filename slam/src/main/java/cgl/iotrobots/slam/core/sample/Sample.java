package cgl.iotrobots.slam.core.sample;

import cgl.iotrobots.slam.core.gridfastsalm.GridSlamProcessor;
import cgl.iotrobots.slam.core.sensor.OdometrySensor;
import cgl.iotrobots.slam.core.sensor.RangeSensor;
import cgl.iotrobots.slam.core.sensor.Sensor;
import cgl.iotrobots.slam.core.utils.OrientedPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.Map;

public class Sample {
    private static Logger LOG = LoggerFactory.getLogger(Sample.class);

    GridSlamProcessor gsp_;
    RangeSensor gsp_laser_;

    double gsp_laser_angle_increment_;
    double angle_min_;
    double angle_max_;
    int gsp_laser_beam_count_;

    int laser_count_;
    int throttle_scans_;

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



    public void init() {
        gsp_ = new GridSlamProcessor();

        throttle_scans_ = 1;

        // gmapping parameters
        maxUrange_ = 0.0;  maxRange_ = 0.0;
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
        temporalUpdate_ = -1.0;
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

    public boolean initMapper()
    {
//        laser_frame_ = scan.header.frame_id;
        // Get the laser's pose, relative to base.
        tf::Stamped<tf::Pose> ident;
        tf::Stamped<tf::Transform> laser_pose;
        ident.setIdentity();
        ident.frame_id_ = laser_frame_;
        ident.stamp_ = scan.header.stamp;
        try
        {
            tf_.transformPose(base_frame_, ident, laser_pose);
        }
        catch(tf::TransformException e)
        {
            LOG.warn("Failed to compute laser pose, aborting initialization (%s)",
                    e.what());
            return false;
        }

        // create a point 1m above the laser position and transform it into the laser-frame
        tf::Vector3 v;
        v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
        tf::Stamped<tf::Vector3> up(v, scan.header.stamp,
                base_frame_);
        try
        {
            tf_.transformPoint(laser_frame_, up, up);
            ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
        }
        catch(tf::TransformException& e)
        {
            ROS_WARN("Unable to determine orientation of laser: %s",
                    e.what());
            return false;
        }

        // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
        if (fabs(fabs(up.z()) - 1) > 0.001)
        {
            ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                    up.z());
            return false;
        }

        gsp_laser_beam_count_ = scan.ranges.size();

        int orientationFactor;
        if (up.z() > 0)
        {
            orientationFactor = 1;
            ROS_INFO("Laser is mounted upwards.");
        }
        else
        {
            orientationFactor = -1;
            ROS_INFO("Laser is mounted upside down.");
        }

        angle_min_ = orientationFactor * scan.angle_min;
        angle_max_ = orientationFactor * scan.angle_max;
        gsp_laser_angle_increment_ = orientationFactor * scan.angle_increment;
        LOG.debug("Laser angles top down in laser-frame: min: %.3f max: %.3f inc: %.3f", angle_min_, angle_max_, gsp_laser_angle_increment_);

        OrientedPoint<Double> gmap_pose = new OrientedPoint<Double>(0.0, 0.0, 0.0);

        // setting maxRange and maxUrange here so we can set a reasonable default
        ros::NodeHandle private_nh_("~");
        if(!private_nh_.getParam("maxRange", maxRange_))
            maxRange_ = scan.range_max - 0.01;
        if(!private_nh_.getParam("maxUrange", maxUrange_))
            maxUrange_ = maxRange_;

        // The laser must be called "FLASER".
        // We pass in the absolute value of the computed angle increment, on the
        // assumption that GMapping requires a positive angle increment.  If the
        // actual increment is negative, we'll swap the order of ranges before
        // feeding each scan to GMapping.
        gsp_laser_ = new RangeSensor("FLASER",
            gsp_laser_beam_count_,
            Math.abs(gsp_laser_angle_increment_),
            gmap_pose,
            0.0,
            maxRange_);

        Map<String, Sensor> smap = new HashMap<String, Sensor>();
        smap.put(gsp_laser_.getName(), gsp_laser_);
        gsp_.setSensorMap(smap);

        gsp_odom_ = new OdometrySensor(odom_frame_);
        ROS_ASSERT(gsp_odom_);


        /// @todo Expose setting an initial pose
        OrientedPoint<Double> initialPose = new OrientedPoint<Double>(0.0, 0.0, 0.0);
        if(!getOdomPose(initialPose, scan.header.stamp))
        {
            LOG.warn("Unable to determine inital pose of laser! Starting point will be set to zero.");
            initialPose = new OrientedPoint<Double>(0.0, 0.0, 0.0);
        }

        gsp_.setMatchingParameters(maxUrange_, maxRange_, sigma_,
                kernelSize_, lstep_, astep_, iterations_,
                lsigma_, ogain_, lskip_);

        gsp_.setMotionModelParameters(srr_, srt_, str_, stt_);
        gsp_.setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
        gsp_.getMatcher().setupdatePeriod(temporalUpdate_);
        gsp_.getMatcher().setgenerateMap(false);
        gsp_.init(particles_, xmin_, ymin_, xmax_, ymax_, delta_, initialPose);
        gsp_.getMatcher().setllsamplerange(llsamplerange_);
        gsp_.getMatcher().setllsamplestep(llsamplestep_);
        /// @todo Check these calls; in the gmapping gui, they use
        /// llsamplestep and llsamplerange intead of lasamplestep and
        /// lasamplerange.  It was probably a typo, but who knows.
        gsp_.getMatcher().setlasamplerange(lasamplerange_);
        gsp_.getMatcher().setlasamplestep(lasamplestep_);
        gsp_.getMatcher().setminimumScore(minimum_score_);

        // Call the sampling function once to set the seed.
        GMapping::sampleGaussian(1,time(NULL));

        LOG.info("Initialization complete");

        return true;
    }
}
