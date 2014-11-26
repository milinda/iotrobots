import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Vector3;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class utils {
private static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
public static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);

    public static Point3d toROSCoordinate(Point3d p){
        return new Point3d(p.getX(),-p.getZ(),p.getY());
    }

    public static Vector3d toROSCoordinate(Vector3d p){
        return new Vector3d(p.getX(),-p.getZ(),p.getY());
    }

    public static Pose getPose(Point3d p3d,Quat4d quat4d){
        Pose pose=messageFactory.newFromType(Pose._TYPE);
        Point3d p3dROS=toROSCoordinate(p3d);

        pose.getPosition().setX(p3dROS.getX());
        pose.getPosition().setY(p3dROS.getY());
        // set z to zero, previously this is the z position of the robot center
        pose.getPosition().setZ(0);

        pose.getOrientation().setZ(quat4d.getY());
        pose.getOrientation().setW(quat4d.getW());
        return pose;
    }

    public static Vector3 getTwist(Vector3d vel){
        Vector3 twist=messageFactory.newFromType(Vector3._TYPE);
        Vector3d velROS=toROSCoordinate(vel);
        twist.setX(velROS.getX());
        twist.setY(velROS.getY());
        twist.setZ(velROS.getZ());
        return twist;
    }
}