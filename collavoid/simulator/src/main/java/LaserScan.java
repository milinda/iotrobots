import org.ros.message.Time;
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

    public LaserScan(double radius, double angle, int nbsensors, double maxRange, int updateFreq) {
        this.radius = radius;
        this.height = 0;
        // angle is arranged symmetrically around x axis
        // compute angles ,positions , directions
        positions = new Vector3d[nbsensors];
        directions = new Vector3d[nbsensors];
        Vector3d frontPos = new Vector3d(radius, 0, 0);
        Vector3d frontDir = new Vector3d(maxRange, 0, 0);
        angles = new double[nbsensors];
        Transform3D transform = new Transform3D();
        double step = angle / ((double) nbsensors - 1);
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

    public void getScan(List<Point3d> res) {
        //in robot base frame
        double d;
        res.clear();
        for (int i = 0; i < sensors.getNumSensors(); i++) {
            if (Double.isFinite(sensors.getMeasurement(i))) {
                d = sensors.getMeasurement(i) + this.radius;
                res.add(new Point3d(d * Math.cos(angles[i]), height, d * Math.sin(-angles[i])));
            }
        }
    }

    public boolean getLaserscanPointCloud2(PointCloud2 pc2) {
        List<Point3d> scan = new ArrayList<Point3d>();
        getScan(scan);
        if (scan.size() == 0) {
            return false;
        } else {
            utils.toPointCloud2(pc2, scan);
            return true;
        }
    }

    public void setHeight(double h) {
        this.height = h;
    }
}
