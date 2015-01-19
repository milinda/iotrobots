package cgl.iotrobots.collavoid.simulator;

import cgl.iotrobots.collavoid.commons.rmqmsg.PointCloud2_;
import sensor_msgs.PointCloud2;
import simbad.sim.RangeSensorBelt;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;


public class LaserScan {
    private double angles[];
    /**
     * position of each sensor relative to center
     */
    private Vector3d positions[];
    /**
     * direction vector of each sensor - relative to sensor position. In robot frame
     */
    private Vector3d directions[];

    private RangeSensorBelt sensors;
    //sensor height
    private double height;
    //sensor belt radius
    private double radius;

    private double minRange,maxRange;

    // original
    public LaserScan(double radius, double angle, int nbsensors, double minRange_, double maxRange_, int updateFreq) {
        this.maxRange = maxRange_;
        this.minRange = minRange_;
        this.radius = radius;
        this.height = 0;
        // angle is arranged symmetrically around x axis
        // compute angles ,positions , directions
        positions = new Vector3d[nbsensors];
        directions = new Vector3d[nbsensors];
        Vector3d frontPos = new Vector3d(radius, 0, 0);
        Vector3d frontDir = new Vector3d(maxRange-radius, 0, 0);
        angles = new double[nbsensors];
        Transform3D transform = new Transform3D();
        double step = angle / ((double) nbsensors - 1);
        // arranged from right to left
        for (int i = 0; i < nbsensors; i++) {
            angles[i] = -angle / 2 + i * step;
            transform.setIdentity();
            transform.rotY(angles[i]);
            Vector3d pos = new Vector3d(frontPos);
            transform.transform(pos);
            positions[i] = pos;
            Vector3d dir = new Vector3d(frontDir);
            transform.transform(dir);
            directions[i] = dir;
        }
        sensors = new RangeSensorBelt(positions, directions, RangeSensorBelt.TYPE_LASER, 0);
        sensors.setUpdatePerSecond(updateFreq);
    }


    public RangeSensorBelt getSensor() {
        return sensors;
    }

    public void setHeight(double h) {
        this.height = h;
    }

    public void getScan(List<Point3d> res) {
        //in robot base frame
        double d;
        res.clear();
        for (int i = 0; i < sensors.getNumSensors(); i++) {
            if (Double.isFinite(sensors.getMeasurement(i))) {
                d = sensors.getMeasurement(i) + this.radius;
                double x=d * Math.cos(angles[i]);
                double z=d * Math.sin(-angles[i]);
                if (x>=this.minRange)
                res.add(new Point3d(x, height,z ));
            }
        }
    }

    public void getLaserscanPointCloud2(PointCloud2 pc2, Transform3D transform3D) {
        List<Point3d> scan = new ArrayList<Point3d>();
        getScan(scan);
        // transform to map or global frame
        for (int i = 0; i < scan.size(); i++) {
            transform3D.transform(scan.get(i));
        }
        utilsSim.toPointCloud2(pc2, scan);
    }

    //no ros
    public void getLaserscanPointCloud2(PointCloud2_ pc2, Transform3D transform3D) {
        List<Point3d> scan = new ArrayList<Point3d>();
        getScan(scan);
        // transform to map or global frame
        for (int i = 0; i < scan.size(); i++) {
            transform3D.transform(scan.get(i));
        }
        utilsSim.toPointCloud2(pc2, scan);
    }
}
