package cgl.iotrobots.collavoid.rosplanners.LocalPlanner;

import cgl.iotrobots.collavoid.rosplanners.utils.Vector2;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import nav_msgs.Odometry;
import nav_msgs.Path;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import java.util.List;

public class LPutils {
//TODO
    public static void publishPlanDeprecated(ConnectedNode node, final List<PoseStamped> path, final Publisher pub){
        Path pathmsg=node.getTopicMessageFactory().newFromType(Path._TYPE);
        pathmsg.setPoses(path);
        pathmsg.setHeader(path.get(0).getHeader());
        pub.publish(pathmsg);
    }


    public static double getYaw(Quaternion q){
        double q0=q.getX();
        double q1=q.getY();
        double q2=q.getZ();
        double q3=q.getW();
        //refer to roll in http://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        return Math.atan2(2.0*(q0*q1 + q3*q2), q3*q3 + q0*q0 - q1*q1 - q2*q2);
    }

    public static double getGoalPositionDistance(Pose pose,double goalx,double goaly){
       return Vector2.abs(new Vector2(pose.getPosition().getX() - goalx, pose.getPosition().getY() - goaly));
    }

    public static double getGoalOrientationAngleDifference(Pose pose,double goalth){
        return getYaw(pose.getOrientation())-goalth;
    }

    public static boolean stopped(Odometry base_odom,double rot_stopped_velocity_,double trans_stopped_velocity_){
        Vector3 angular,linear;
        linear=base_odom.getTwist().getTwist().getLinear();
        angular=base_odom.getTwist().getTwist().getAngular();
        double lx=linear.getX(),ly=linear.getY(),lz=linear.getZ();
        double ax=angular.getX(),ay=angular.getY(),az=angular.getZ();
        return Math.sqrt(lx*lx+ly*ly+lz*lz)<trans_stopped_velocity_&&Math.sqrt(ax*ax+ay*ay+az*az)<rot_stopped_velocity_;
    }
}
