package cgl.iotrobots.collavoid.LocalPlanner;

import cgl.iotrobots.collavoid.utils.Vector2;
import costmap_2d.VoxelGrid;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import nav_msgs.Odometry;
import org.ros.node.topic.Publisher;
import org.ros.rosjava.tf.pubsub.TransformListener;

import java.util.List;

/**
 * Created by hjh on 11/4/14.
 */
public class LPutils {
//TODO
    public static void publishPlan(final List<PoseStamped> path,final Publisher pub){

    }


    public static double getYaw(Quaternion q){
        double q0=q.getX();
        double q1=q.getY();
        double q2=q.getZ();
        double q3=q.getW();
        //refer to http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        return Math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
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
