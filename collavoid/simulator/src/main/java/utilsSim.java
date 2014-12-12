import geometry_msgs.Pose;
import geometry_msgs.Vector3;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;
import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;

import javax.vecmath.*;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class utilsSim {

    private static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    public static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);
    //public static Transform3D tftoROS=new Transform3D(new Quat4d(-1,0,0,Math.cos(Math.PI/4)),new Vector3d(0,0,0),1);
    public static Point3d toROSCoordinate(Point3d p) {
        return new Point3d(p.getX(), -p.getZ(), p.getY());
    }

    public static void toROSCoordinate(Vector3d p) {
        p.set(p.getX(), -p.getZ(), p.getY());
    }

    public static void toROSCoordinate(Quat4d q) {
            q.set(q.getX(),-q.getZ(),q.getY(),q.getW());
    }

    public static Pose getPose(Point3d p3d, Quat4d quat4d) {
        Pose pose = messageFactory.newFromType(Pose._TYPE);
        Point3d p3dROS = toROSCoordinate(p3d);

        pose.getPosition().setX(p3dROS.getX());
        pose.getPosition().setY(p3dROS.getY());
        // set z to zero, previously this is the z position of the robot center
        pose.getPosition().setZ(0);

        pose.getOrientation().setZ(quat4d.getY());
        pose.getOrientation().setW(quat4d.getW());
        return pose;
    }

    public static Vector3 getTwist(Vector3d vel) {
        Vector3 twist = messageFactory.newFromType(Vector3._TYPE);
        Vector3d velROS = new Vector3d(vel);
        toROSCoordinate(velROS);
        twist.setX(velROS.getX());
        twist.setY(velROS.getY());
        twist.setZ(velROS.getZ());
        return twist;
    }


    public static void toPointCloud2(PointCloud2 pc2, List<Point3d> scan) {
        List<PointField> lPointField = new ArrayList<PointField>();
        String fieldName = "xyz";

        for (int i = 0; i < 3; i++) {
            PointField pf = utilsSim.messageFactory.newFromType(PointField._TYPE);
            pf.setName(fieldName.substring(i, i + 1));
            pf.setOffset(i * 4);
            pf.setDatatype(PointField.FLOAT32);
            pf.setCount(1);
            lPointField.add(pf);
        }
        pc2.setHeight(1);
        pc2.setWidth(scan.size());
        pc2.setFields(lPointField);
        pc2.setIsBigendian(false);
        pc2.setPointStep(4 * lPointField.size());
        pc2.setRowStep(pc2.getPointStep() * scan.size());
        pc2.setIsDense(false);

        ChannelBuffer buffer = ChannelBuffers.buffer(ByteOrder.LITTLE_ENDIAN, pc2.getRowStep());
        buffer.clear();

        Point3d p3d;
        for (int i = 0; i < scan.size(); i++) {
            //in ros coordinate
            double[] pt = new double[lPointField.size()];
            p3d = toROSCoordinate(scan.get(i));
            p3d.get(pt);
            for (int j = 0; j < lPointField.size(); j++) {
                buffer.writeFloat((float) pt[j]);
            }
        }
        if (scan.size()==0)
            pc2.getData().clear();
        else
            pc2.setData(buffer);
    }

    public static void toPointCloud2(PointCloud2 pc2, List<Point3d> scan, MainSimulator.Robot robot) {
        pc2.getHeader().setFrameId(robot.globalFrame);
        toPointCloud2(pc2,scan);
    }

    public static void toPointCloud2(PointCloud2 pc2, List<Point3d> scan, MainSimulatorWithPlanner.Robot robot) {
        pc2.getHeader().setFrameId(robot.globalFrame);
        toPointCloud2(pc2,scan);
    }

    public static void Vector3ToPoint3d(Vector3 in, Point3d out) {
        out.set(in.getX(), in.getY(), in.getZ());
    }

    public static void Point3dToVector3(Point3d in, Vector3 out) {
        out.setX(in.getX());
        out.setY(in.getY());
        out.setZ(in.getZ());
    }

    public static double getGaussianNoise(double mean, double variance) {
        Random r = new java.util.Random();
        double noise = r.nextGaussian() * Math.sqrt(variance) + mean;
        return noise;
    }
}