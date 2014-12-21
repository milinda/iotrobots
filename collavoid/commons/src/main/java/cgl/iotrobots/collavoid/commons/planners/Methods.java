package cgl.iotrobots.collavoid.commons.planners;

import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Pose_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Vector3d_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Vector4d_;

public class Methods {

    // local planner related
    public static double getYaw(Vector4d_ q) {
        double q0 = q.getX();
        double q1 = q.getY();
        double q2 = q.getZ();
        double q3 = q.getW();
        //refer to roll in http://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        return Math.atan2(2.0 * (q0 * q1 + q3 * q2), q3 * q3 + q0 * q0 - q1 * q1 - q2 * q2);
    }

    public static double getGoalPositionDistance(Pose_ pose, double goalx, double goaly) {
        return Vector2.abs(new Vector2(pose.getPosition().getX() - goalx, pose.getPosition().getY() - goaly));
    }

    public static double getGoalOrientationAngleDifference(Pose_ pose, double goalth) {
        return getYaw(pose.getOrientation()) - goalth;
    }

    public static boolean stopped(Odometry_ base_odom, double rot_stopped_velocity_, double trans_stopped_velocity_) {
        Vector3d_ angular, linear;
        linear = base_odom.getTwist().getLinear();
        angular = base_odom.getTwist().getAngular();
        double lx = linear.getX(), ly = linear.getY(), lz = linear.getZ();
        double ax = angular.getX(), ay = angular.getY(), az = angular.getZ();
        return Math.sqrt(lx * lx + ly * ly + lz * lz) < trans_stopped_velocity_ && Math.sqrt(ax * ax + ay * ay + az * az) < rot_stopped_velocity_;
    }

    // angel related
    public static double normalize_angle_positive(double angle) {
        return ((angle % (2.0 * Math.PI)) + 2.0 * Math.PI) % (2.0 * Math.PI);//why mod twice??
    }

    public static double normalize_angle(double angle) {
        double a = normalize_angle_positive(angle);
        if (a > Math.PI)
            a -= 2.0 * Math.PI;
        return a;
    }

    public static double shortest_angular_distance(double from, double to) {
        double result = normalize_angle_positive(normalize_angle_positive(to) - normalize_angle_positive(from));

        if (result > Math.PI)
            // If the result > 180,
            // It's shorter the other way.
            result = -(2.0 * Math.PI - result);

        return normalize_angle(result);
    }


    // line related
    public static Vector2 projectPointOnLine(final Vector2 pointLine, final Vector2 dirLine, final Vector2 point) {
        double r = (Vector2.dotProduct(Vector2.minus(point, pointLine), dirLine)) / Vector2.absSqr(dirLine);
        return Vector2.plus(pointLine, Vector2.mul(dirLine, r));
    }

    /*!
     *  @brief      Computes the squared distance from a line segment with the
     *              specified endpoints to a specified point.
     *  @param      a               The first endpoint of the line segment.
     *  @param      b               The second endpoint of the line segment.
     *  @param      c               The point to which the squared distance is to
     *                              be calculated.
     *  @returns    The squared distance from the line segment to the point.
     */
    public static double distSqPointLineSegment(final Vector2 a, final Vector2 b, final Vector2 c) {
        final double r = Vector2.dotProduct(Vector2.minus(c, a), Vector2.minus(b, a)) / Vector2.absSqr(Vector2.minus(b, a));

        if (r < 0.0f) {
            return Vector2.absSqr(Vector2.minus(c, a));
        } else if (r > 1.0f) {
            return Vector2.absSqr(Vector2.minus(c, b));
        } else {
            return Vector2.absSqr(Vector2.minus(c, Vector2.plus(a, Vector2.mul(Vector2.minus(b, a), r))));
        }
    }

    /*!
     *  @brief      Computes the sign from a line connecting the
     *              specified points to a specified point.
     *  @param      a               The first point on the line.
     *  @param      b               The second point on the line.
     *  @param      c               The point to which the signed distance is to
     *                              be calculated.
     *  @returns    Positive when the point c lies to the left of the line ab.
     */
    public static double signedDistPointToLineSegment(final Vector2 a, final Vector2 b, final Vector2 c) {
        return Vector2.det(Vector2.minus(a, c), Vector2.minus(b, a));
    }

    public static double left(final Vector2 pointLine, final Vector2 dirLine, final Vector2 point) {
        return signedDistPointToLineSegment(pointLine, Vector2.plus(pointLine, dirLine), point);
    }

    public static boolean leftOf(final Vector2 pointLine, final Vector2 dirLine, final Vector2 point) {
        return signedDistPointToLineSegment(pointLine, Vector2.plus(pointLine, dirLine), point) > Parameters.EPSILON;
    }

    public static boolean rightOf(final Vector2 pointLine, final Vector2 dirLine, final Vector2 point) {
        return signedDistPointToLineSegment(pointLine, Vector2.plus(pointLine, dirLine), point) < -Parameters.EPSILON;
    }

    public static double sign(double x) {
        return x < 0.0 ? -1.0 : 1.0;
    }
}
