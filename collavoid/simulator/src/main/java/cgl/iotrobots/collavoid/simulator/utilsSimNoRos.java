package cgl.iotrobots.collavoid.simulator;

import cgl.iotrobots.collavoid.commons.rmqmsg.Pose_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Vector3d_;
import geometry_msgs.Vector3;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.Random;

public class utilsSimNoRos {
    public static Point3d toROSCoordinate(Point3d p) {
        return new Point3d(p.getX(), -p.getZ(), p.getY());
    }

    public static void toROSCoordinate(Vector3d p) {
        p.set(p.getX(), -p.getZ(), p.getY());
    }

    public static void toROSCoordinate(Quat4d q) {
        q.set(q.getX(), -q.getZ(), q.getY(), q.getW());
    }

    public static Pose_ getPose(Point3d p3d, Quat4d quat4d) {
        Pose_ pose = new Pose_();
        Point3d p3dROS = toROSCoordinate(p3d);

        pose.getPosition().setX(p3dROS.getX());
        pose.getPosition().setY(p3dROS.getY());
        // set z to zero, previously this is the z position of the robot center
        pose.getPosition().setZ(0);

        Quat4d q = new Quat4d(quat4d);
        toROSCoordinate(q);
        pose.getOrientation().setZ(q.getZ());
        pose.getOrientation().setW(q.getW());
        return pose;
    }


    public static Vector3d_ getTwist(Vector3d vel) {
        Vector3d_ twist = new Vector3d_();
        Vector3d velROS = new Vector3d(vel);
        toROSCoordinate(velROS);
        twist.setX(velROS.getX());
        twist.setY(velROS.getY());
        twist.setZ(velROS.getZ());
        return twist;
    }

    public static double getGaussianNoise(double mean, double variance) {
        Random r = new java.util.Random();
        double noise = r.nextGaussian() * Math.sqrt(variance) + mean;
        return noise;
    }
}
