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

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

public class utils {

    private static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    public static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);

    public static Point3d toROSCoordinate(Point3d p) {
        return new Point3d(p.getX(), -p.getZ(), p.getY());
    }

    public static Vector3d toROSCoordinate(Vector3d p) {
        return new Vector3d(p.getX(), -p.getZ(), p.getY());
    }

    public static Quat4d toROSCoordinate(Quat4d q){
        return new Quat4d(q.getX(),-q.getZ(),q.getY(),q.getW());
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
        Vector3d velROS = toROSCoordinate(vel);
        twist.setX(velROS.getX());
        twist.setY(velROS.getY());
        twist.setZ(velROS.getZ());
        return twist;
    }


    public static void toPointCloud2(PointCloud2 pc2, List<Point3d> scan) {
        List<PointField> lPointField = new ArrayList<PointField>();
        String fieldName = "xyz";

        for (int i = 0; i < 3; i++) {
            PointField pf = utils.messageFactory.newFromType(PointField._TYPE);
            pf.setName(fieldName.substring(i, i+1));
            pf.setOffset(i * 4);
            pf.setDatatype(PointField.FLOAT32);
            pf.setCount(1);
            lPointField.add(pf);
        }
        pc2.setHeight(1);
        pc2.setWidth(scan.size());
        pc2.setFields(lPointField);
        pc2.setIsBigendian(false);
        pc2.setPointStep(4 * 3);
        pc2.setRowStep(pc2.getPointStep() * scan.size());
        pc2.setIsDense(false);

        ChannelBuffer buffer = ChannelBuffers.buffer(ByteOrder.LITTLE_ENDIAN,pc2.getRowStep());
        buffer.clear();

        for (int i = 0; i < scan.size(); i++) {
            //in ros coordinate
            Point3d p3d=new Point3d();
            p3d=toROSCoordinate(scan.get(i));
            buffer.setFloat(i*12,(float) p3d.getX());
            buffer.setFloat(i*12+4,(float) p3d.getY());
            buffer.setFloat(i*12+8,(float) p3d.getZ());
//            pc2.getData().setFloat(i*12,(float) p3d.getX());
//            pc2.getData().setFloat(i*12+4,(float) p3d.getY());
//            pc2.getData().setFloat(i*12+8,(float) p3d.getZ());
        }
        pc2.setData(buffer);

    }
}