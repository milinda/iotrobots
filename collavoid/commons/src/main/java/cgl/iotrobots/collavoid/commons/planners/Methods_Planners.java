package cgl.iotrobots.collavoid.commons.planners;

/* Some of the methods are based on multi_robot_collision_avoidance on wiki ros, license information: refer to
 http://wiki.ros.org/action/login/multi_robot_collision_avoidance */

import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Methods_Planners implements Serializable {
    private static Logger logger = LoggerFactory.getLogger(Methods_Planners.class);

    public static List<Vector2> rotateFootprint(final List<Vector2> footprint, double angle) {
        List<Vector2> result = new ArrayList<Vector2>();
        for (int i = 0; i < footprint.size(); ++i) {
            Vector2 rotated = Vector2.rotateVectorByAngle(footprint.get(i), angle);
            result.add(rotated);
        }
        return result;
    }

    // local planner related
    public static double getYaw(Vector4d_ q) {
        double q0 = q.getX();
        double q1 = q.getY();
        double q2 = q.getZ();
        double q3 = q.getW();
        //refer to roll in http://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        return Math.atan2(2.0 * (q0 * q1 + q3 * q2), q3 * q3 + q0 * q0 - q1 * q1 - q2 * q2);
    }

    public static Vector4d_ getQuaternion(Vector3d_ vec, double theta) {
        double theta_normalized = normalize_angle(theta);
        double w = Math.cos(normalize_angle(theta) / 2);
        double s = Math.sqrt(1 - w * w) / vec.length();
        if (theta_normalized < 0)
            s = -s;
        vec.scale(s);
        return new Vector4d_(vec.getX(), vec.getY(), vec.getZ(), w);
    }

    public static double getPositionDistance(Pose_ p1, Pose_ p2) {
        Vector3d_ v1 = p1.getPosition();
        Vector3d_ v2 = p2.getPosition();
        double x1 = v1.getX();
        double y1 = v1.getY();
        double z1 = v1.getZ();
        double x2 = v2.getX();
        double y2 = v2.getY();
        double z2 = v2.getZ();
        return Math.sqrt(
                (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2)
        );
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
        return ((angle % (2.0 * Math.PI)) + 2.0 * Math.PI) % (2.0 * Math.PI);
    }

    // from -PI to PI
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
        return Vector2.plus(pointLine, Vector2.scale(dirLine, r));
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
            return Vector2.absSqr(Vector2.minus(c, Vector2.plus(a, Vector2.scale(Vector2.minus(b, a), r))));
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

    public static List<Vector2> minkowskiSumConvexHull(final List<Vector2> polygon1, final List<Vector2> polygon2) {
        List<Vector2> result = new ArrayList<Vector2>();
        if (polygon1.size() == 0 && polygon2.size() == 0) {
            return result;
        } else if (polygon1.size() == 0) {
            for (Vector2 pt : polygon2) {
                result.add(new Vector2(pt.getX(), pt.getY()));
            }
            return result;
        } else if (polygon2.size() == 0) {
            for (Vector2 pt : polygon1) {
                result.add(new Vector2(pt.getX(), pt.getY()));
            }
            return result;
        }
        ConvexHullPoint[] convex_hull = new ConvexHullPoint[polygon1.size() * polygon2.size()];

        int n = 0;
        for (int i = 0; i < polygon1.size(); i++) {
            for (int j = 0; j < polygon2.size(); j++) {
                ConvexHullPoint p = new ConvexHullPoint();
                p.setPoint(Vector2.plus(polygon1.get(i), polygon2.get(j)));
                convex_hull[n++] = p;
            }
        }
        convex_hull = convexHull(convex_hull, false);
        for (int i = 0; i < convex_hull.length; i++) {
            result.add(new Vector2(convex_hull[i].getX(), convex_hull[i].getY()));
        }
        return result;
    }

    public static ConvexHullPoint[] convexHull(ConvexHullPoint[] P, boolean sorted) {
        int n = P.length, k = 0;
        ConvexHullPoint[] result = new ConvexHullPoint[2 * n];

        if (!sorted)
            Arrays.sort(P, new Comparators.VectorsLexigraphicComparator());

        // Build lower hull,
        for (int i = 0; i < n; i++) {
            while (k >= 2 && Vector2.det(
                    result[k - 2].getX() - result[k - 1].getX(),
                    result[k - 2].getY() - result[k - 1].getY(),
                    P[i].getX() - result[k - 1].getX(),
                    P[i].getY() - result[k - 1].getY()
            ) <= 0) k--;
            ConvexHullPoint pt = new ConvexHullPoint(P[i].getX(), P[i].getY());
            result[k++] = pt;
        }

        // Build upper hull
        for (int i = n - 2, t = k + 1; i >= 0; i--) {
            while (k >= t && Vector2.det(
                    result[k - 2].getX() - result[k - 1].getX(),
                    result[k - 2].getY() - result[k - 1].getY(),
                    P[i].getX() - result[k - 1].getX(),
                    P[i].getY() - result[k - 1].getY()
            ) <= 0) k--;
            ConvexHullPoint pt = new ConvexHullPoint(P[i].getX(), P[i].getY());
            result[k++] = pt;
        }


        //resize list
        ConvexHullPoint[] resultnew = new ConvexHullPoint[k];
        System.arraycopy(result, 0, resultnew, 0, k);

        return resultnew;
    }

    public static class NHORCA {
        //from orca
        //in velocity space
        public static void addNHConstraints(Agent agent) {
            if (agent.holo_robot_)
                return;

            double min_error = agent.minErrorHolo;
            double max_error = agent.maxErrorHolo;
            double error = max_error;
            double v_max_ang = NHORCA.vMaxAng(agent);

            if (agent.min_dist < 2.0 * agent.footprint_radius_ + agent.cur_loc_unc_radius_) {
                error = (max_error - min_error) /
                        (Math.pow(2 * (agent.footprint_radius_ + agent.cur_loc_unc_radius_), 2)) *
                        Math.pow(agent.min_dist, 2) + min_error; // how much error do i allow?
                if (agent.min_dist < 0) {
                    error = min_error;
                }
            }
            agent.cur_allowed_nh_error_ = 1.0 / 3.0 * agent.cur_allowed_nh_error_ + 2.0 / 3.0 * error;

            double speed_ang = Math.atan2(agent.prefVelociy.getY(), agent.prefVelociy.getX());
            double dif_ang = shortest_angular_distance(agent.position.getHeading(), speed_ang);
            //calculate possible tracking holomonic robot speed range
            if (Math.abs(dif_ang) > Math.PI / 2.0) { // || cur_allowed_nh_error_ < 2.0 * min_error) {
                double max_track_speed = NHORCA.calculateMaxTrackSpeedAngle(
                        agent.time_to_holo_,
                        Math.PI / 2.0,
                        agent.cur_allowed_nh_error_,
                        agent.max_vel_x_,
                        agent.max_vel_th_,
                        v_max_ang);
                // tracking errors are in velocity space?
                if (max_track_speed <= 2 * min_error) {
                    max_track_speed = 2 * min_error;
                }
                NHORCA.addMovementConstraintsDiffSimple(
                        max_track_speed,
                        agent.position.getHeading(),
                        agent.addOrcaLines);
            } else {
                NHORCA.addMovementConstraintsDiff(
                        agent.cur_allowed_nh_error_,
                        agent.time_to_holo_,
                        agent.max_vel_x_,
                        agent.max_vel_th_,
                        agent.position.getHeading(),
                        v_max_ang,
                        agent.addOrcaLines);
            }
            agent.max_speed_x_ = NHORCA.vMaxAng(agent);

        }


        public static void addMovementConstraintsDiffSimple(double max_track_speed, double heading, List<Line> additional_orca_lines) {
            Line maxVel1;
            Line maxVel2;

            //if angle difference is more than PI/2 then set angle difference to PI/2 and
            //this is the max holo velocity that can track.
            Vector2 dir = new Vector2(Math.cos(heading), Math.sin(heading));
            Vector2 pt = Vector2.scale(new Vector2(-dir.getY(), dir.getX()), 0.95 * max_track_speed);
            maxVel1 = new Line(pt, Vector2.negative(dir));
            pt = Vector2.scale(new Vector2(-dir.getY(), dir.getX()), -max_track_speed);
            maxVel2 = new Line(pt, dir);
            maxVel1.setType("NHConstraints");
            maxVel2.setType("NHConstraints");
            additional_orca_lines.add(maxVel1);
            additional_orca_lines.add(maxVel2);
        }

        //from orca
        public static void addMovementConstraintsDiff(
                double error,
                double T,
                double max_vel_x,
                double max_vel_th,
                double heading,
                double v_max_ang,
                List<Line> additional_orca_lines) {
            // the max tracking angle
            double min_theta = Math.PI / 2.0;
            double max_track_speed = calculateMaxTrackSpeedAngle(T, min_theta, error, max_vel_x, max_vel_th, v_max_ang);

            //normal vector of heading, clock wise
            Vector2 first_point = new Vector2(Math.cos(heading - min_theta), Math.sin(heading - min_theta));
            first_point.scale(max_track_speed);

            double steps = 10.0;
            double step_size = -Math.PI / steps;
            // set all holo velocity that can be tracked for angle from heading-PI/2 to heading+PI/2
            for (int i = 1; i <= (int) steps; i++) {
                Line line;
                double cur_ang = min_theta + i * step_size;
                Vector2 second_point = new Vector2(Math.cos(heading - cur_ang), Math.sin(heading - cur_ang));
                double track_speed = calculateMaxTrackSpeedAngle(T, cur_ang, error, max_vel_x, max_vel_th, v_max_ang);
                second_point.setVector2(Vector2.scale(second_point, track_speed));

                line = new Line(first_point, Vector2.normalize(Vector2.minus(second_point, first_point)));
                line.setType("NHConstraints");
                additional_orca_lines.add(line);
                first_point.setVector2(second_point);
            }
        }

        public static void addAccelerationConstraintsXY(
                double max_vel_x,
                double acc_lim_x,
                double max_vel_y,
                double acc_lim_y,
                final Vector2 cur_vel_agent,
                double heading,
                double sim_period,
                boolean holo_robot,
                List<Line> additional_orca_lines) {
            Vector2 cur_vel = new Vector2(cur_vel_agent.getX(), cur_vel_agent.getY());
            double max_lim_x, max_lim_y, min_lim_x, min_lim_y;
            Line line_x_back, line_x_front, line_y_left, line_y_right;

            Vector2 dir_front = new Vector2(Math.cos(heading), Math.sin(heading));
            Vector2 dir_right = new Vector2(dir_front.getY(), -dir_front.getX());
            if (holo_robot) {

                cur_vel = Vector2.rotateVectorByAngle(cur_vel, -heading);

                double cur_x = cur_vel.getX();
                double cur_y = cur_vel.getY();

                max_lim_x = Math.min(max_vel_x, cur_x + sim_period * acc_lim_x);
                max_lim_y = Math.min(max_vel_y, cur_y + sim_period * acc_lim_y);
                min_lim_x = Math.max(-max_vel_x, cur_x - sim_period * acc_lim_x);
                min_lim_y = Math.max(-max_vel_y, cur_y - sim_period * acc_lim_y);

                line_x_front = new Line(Vector2.scale(dir_front, max_lim_x), Vector2.negative(dir_right));

                line_x_back = new Line(Vector2.scale(dir_front, min_lim_x), dir_right);

                line_x_front.setType("accConstraints");
                line_x_back.setType("accConstraints");
                additional_orca_lines.add(line_x_front);
                additional_orca_lines.add(line_x_back);

                line_y_left = new Line(Vector2.scale(Vector2.negative(dir_right), max_lim_y), Vector2.negative(dir_front));

                line_y_right = new Line(Vector2.scale(Vector2.negative(dir_right), min_lim_y), dir_front);

                line_y_left.setType("accConstraints");
                line_y_right.setType("accConstraints");
                additional_orca_lines.add(line_y_left);
                additional_orca_lines.add(line_y_right);
            } else {

                double cur_x = Vector2.abs(cur_vel);
                max_lim_x = Math.min(max_vel_x, cur_x + sim_period * acc_lim_x);
                min_lim_x = Math.max(-max_vel_x, cur_x - sim_period * acc_lim_x);
                line_x_front = new Line(Vector2.scale(dir_front, max_lim_x), Vector2.negative(dir_right));

                line_x_back = new Line(Vector2.scale(dir_front, min_lim_x), dir_right);

                line_x_front.setType("accConstraints");
                line_x_back.setType("accConstraints");
                additional_orca_lines.add(line_x_front);
                additional_orca_lines.add(line_x_back);


            }
        }

        //from original program orca, parameters are corresponding to the NHrobot reference
        //                                                      T        theta_H        e               v_max             w                 v_max_w
        public static double calculateMaxTrackSpeedAngle(double T, double theta, double error, double max_vel_x, double max_vel_th, double v_max_ang) {
            if (Math.abs(theta) <= Parameters.EPSILON)
                return max_vel_x;
            if (Math.abs(theta / T) <= max_vel_th) {// formulation 13 in paper NHORCA
                double vstar_error = NHORCA.calcVstarError(T, theta, error);
                double vstar = calcVstar(error / T, theta);
                if (vstar_error <= v_max_ang) {
                    //return 0;
                    return Math.min(vstar_error / vstar, max_vel_x);
                } else {
                    double a, b, g;
                    a = T * T;
                    b = beta(T, theta, v_max_ang);
                    g = gamma(T, theta, error, v_max_ang);
                    //return 1;
                    //different from the reference not sure right or wrong so stick to the original reference
                    //return Math.min((-b + Math.sqrt(Math.pow(b, 2) - 4 * Math.pow(a, 2) * g)) / (2.0 * g), max_vel_x);
                    return Math.min((-b + Math.sqrt(Math.pow(b, 2) - 4 * a * g)) / (2.0 * g), max_vel_x);
                }
            } else
                //  return 2;
                //set theta to positive
                return Math.min(sign(theta) * error * max_vel_th / theta, max_vel_x);
        }

        public static double beta(double T, double theta, double v_max_ang) {
            return -((2.0 * T * T * Math.sin(theta)) / theta) * v_max_ang;
        }

        public static double gamma(double T, double theta, double error, double v_max_ang) {
            return ((2.0 * T * T * (1.0 - Math.cos(theta))) / (theta * theta)) * v_max_ang * v_max_ang - error * error;
        }

        public static double calcVstar(double vh, double theta) {
            return vh * ((theta * Math.sin(theta)) / (2.0 * (1.0 - Math.cos(theta))));
        }

        public static double calcVstarError(double T, double theta, double error) {
            return calcVstar(error / T, theta) * Math.sqrt((2.0 * (1.0 - Math.cos(theta))) / (2.0 * (1.0 - Math.cos(theta)) - Math.pow(Math.sin(theta), 2)));
        }

        public static double vMaxAng(Agent agent) {
            //double theoretical_max_v = max_vel_th_ * wheel_base_ / 2.0;
            //return theoretical_max_v - std::abs(base_odom_.twist.twist.angular.z) * wheel_base_/2.0;
            return agent.max_vel_x_; //TODO: fixme
        }


    }

    public static class ClearPath {

        public static final int RAYRAY = 0;
        public static final int RAYSEGMENT = 1;
        public static final int SEGMENTSEGMENT = 2;
        public static final int SEGMENTLINE = 3;
        public static final int RAYLINE = 4;
        public static final int LINELINE = 5;

        public static final int HRVOS = 0;
        public static final int RVOS = 1;
        public static final int VOS = 2;

        // additional constraints include non holonomic robot constraints, acceleration constraints. vos from obstacles are not included in clear path method
        public static Vector2 calculateClearpathVelocity(
                List<VelocitySample> samples,
                final List<VO> truncated_vos,
                final List<Line> additional_constraints,
                final Vector2 pref_vel,
                double max_speed,
                boolean use_truncation) {
            if (samples == null) {
                samples = new ArrayList<VelocitySample>();
            }
            boolean isWithinAllAdditionalConstraints = true;
            if (!isWithinAdditionalConstraints(additional_constraints, pref_vel)) {
                isWithinAllAdditionalConstraints = false;
                for (Line line : additional_constraints) {
                    VelocitySample pref_vel_sample = new VelocitySample();
                    // nearest point on vo line to pref vel
                    pref_vel_sample.setVelocity(intersectTwoLines(line.getPoint(), line.getDir(),
                            pref_vel,
                            new Vector2(line.getDir().getY(), -line.getDir().getX())));
                    pref_vel_sample.setDistToPrefVel(Vector2.absSqr(Vector2.minus(pref_vel, pref_vel_sample.getVelocity())));
                    samples.add(pref_vel_sample);
                }

                //intersection point of constraint lines
                for (int i = 0; i < additional_constraints.size(); i++) {
                    for (int j = 0; j < additional_constraints.size(); j++) {
                        addRayVelocitySamples(samples, pref_vel,
                                additional_constraints.get(i).getPoint(), additional_constraints.get(i).getDir(),
                                additional_constraints.get(j).getPoint(), additional_constraints.get(j).getDir(),
                                max_speed, LINELINE);
                    }
                }
            } else {
                VelocitySample pref_vel_sample = new VelocitySample();
                pref_vel_sample.setVelocity(pref_vel);
                pref_vel_sample.setDistToPrefVel(0);
                samples.add(pref_vel_sample);
            }


            boolean isOutsideVOs = true;
            for (int i = 0; i < truncated_vos.size(); i++) {
                if (isInsideVO(truncated_vos.get(i), pref_vel, use_truncation)) {
                    isOutsideVOs = false;

                    VelocitySample leg_projection = new VelocitySample();
                    if (leftOf(truncated_vos.get(i).getPoint(), truncated_vos.get(i).getRelativePosition(), pref_vel)) { //left of centerline, project on left leg
                        leg_projection.setVelocity(intersectTwoLines(truncated_vos.get(i).getPoint(), truncated_vos.get(i).getLeftLegDir(),
                                pref_vel, new Vector2(truncated_vos.get(i).getLeftLegDir().getY(), -truncated_vos.get(i).getLeftLegDir().getX())));
                    } else { //project on right leg
                        leg_projection.setVelocity(intersectTwoLines(truncated_vos.get(i).getPoint(), truncated_vos.get(i).getRightLegDir(),
                                pref_vel, new Vector2(truncated_vos.get(i).getRightLegDir().getY(), -truncated_vos.get(i).getRightLegDir().getX())));
                    }

                    //if(absSqr(leg_projection.velocity) < max_speed) { //only add if below max_speed
                    leg_projection.setDistToPrefVel(Vector2.absSqr(Vector2.minus(pref_vel, leg_projection.getVelocity())));
                    samples.add(leg_projection);
                    //}

                    if (use_truncation) {
                        addRayVelocitySamples(samples, pref_vel, pref_vel, Vector2.negative(truncated_vos.get(i).getRelativePosition()),
                                truncated_vos.get(i).getTruncLeft(), Vector2.minus(truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getTruncLeft()), max_speed, RAYSEGMENT);
                    }
                }
            }

            if (isOutsideVOs && isWithinAllAdditionalConstraints)
                return samples.get(0).getVelocity();

            if (!isOutsideVOs) {
                //calculate all the possible intersection of vo lines and vo cone or truncation
                for (int i = 0; i < truncated_vos.size(); i++) {
                    for (int j = 0; j < additional_constraints.size(); j++) {
                        if (!use_truncation) {
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getPoint(), truncated_vos.get(i).getLeftLegDir(), additional_constraints.get(j).getPoint(), additional_constraints.get(j).getDir(), max_speed, RAYLINE);
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getPoint(), truncated_vos.get(i).getRightLegDir(), additional_constraints.get(j).getPoint(), additional_constraints.get(j).getDir(), max_speed, RAYLINE);
                        } else {
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncLeft(), truncated_vos.get(i).getLeftLegDir(), additional_constraints.get(j).getPoint(), additional_constraints.get(j).getDir(), max_speed, RAYLINE);
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getRightLegDir(), additional_constraints.get(j).getPoint(), additional_constraints.get(j).getDir(), max_speed, RAYLINE);
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncLeft(), Vector2.minus(truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getTruncLeft()),
                                    additional_constraints.get(j).getPoint(), additional_constraints.get(j).getDir(), max_speed, SEGMENTLINE);
                        }
                    }

                    for (int j = i + 1; j < truncated_vos.size(); j++) {

                        if (!use_truncation) {
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getPoint(), truncated_vos.get(i).getLeftLegDir(), truncated_vos.get(j).getPoint(), truncated_vos.get(j).getLeftLegDir(), max_speed, RAYRAY);
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getPoint(), truncated_vos.get(i).getLeftLegDir(), truncated_vos.get(j).getPoint(), truncated_vos.get(j).getRightLegDir(), max_speed, RAYRAY);
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getPoint(), truncated_vos.get(i).getRightLegDir(), truncated_vos.get(j).getPoint(), truncated_vos.get(j).getLeftLegDir(), max_speed, RAYRAY);
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getPoint(), truncated_vos.get(i).getRightLegDir(), truncated_vos.get(j).getPoint(), truncated_vos.get(j).getRightLegDir(), max_speed, RAYRAY);

                        } else {
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncLeft(), truncated_vos.get(i).getLeftLegDir(), truncated_vos.get(j).getTruncLeft(), truncated_vos.get(j).getLeftLegDir(), max_speed, RAYRAY);
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncLeft(), truncated_vos.get(i).getLeftLegDir(), truncated_vos.get(j).getTruncRight(), truncated_vos.get(j).getRightLegDir(), max_speed, RAYRAY);
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getRightLegDir(), truncated_vos.get(j).getTruncLeft(), truncated_vos.get(j).getLeftLegDir(), max_speed, RAYRAY);
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getRightLegDir(), truncated_vos.get(j).getTruncLeft(), truncated_vos.get(j).getRightLegDir(), max_speed, RAYRAY);


                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncLeft(), truncated_vos.get(i).getLeftLegDir(), truncated_vos.get(j).getTruncLeft(),
                                    Vector2.minus(truncated_vos.get(j).getTruncRight(), truncated_vos.get(j).getTruncLeft()), max_speed, RAYSEGMENT); //left trunc
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(j).getTruncLeft(), truncated_vos.get(j).getLeftLegDir(), truncated_vos.get(i).getTruncLeft(),
                                    Vector2.minus(truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getTruncLeft()), max_speed, RAYSEGMENT); //trunc left

                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getRightLegDir(), truncated_vos.get(j).getTruncLeft(),
                                    Vector2.minus(truncated_vos.get(j).getTruncRight(), truncated_vos.get(j).getTruncLeft()), max_speed, RAYSEGMENT); //right trunc
                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(j).getTruncRight(), truncated_vos.get(j).getRightLegDir(), truncated_vos.get(i).getTruncLeft(),
                                    Vector2.minus(truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getTruncLeft()), max_speed, RAYSEGMENT); //trunc right

                            addRayVelocitySamples(samples, pref_vel, truncated_vos.get(i).getTruncLeft(), Vector2.minus(truncated_vos.get(i).getTruncRight(), truncated_vos.get(i).getTruncLeft()),
                                    truncated_vos.get(j).getTruncLeft(), Vector2.minus(truncated_vos.get(j).getTruncRight(), truncated_vos.get(j).getTruncLeft()), max_speed, SEGMENTSEGMENT); //trunc trunc


                        }

                    }
                }
            }

            VelocitySample null_vel_sample = new VelocitySample();
            null_vel_sample.setVelocity(new Vector2(0, 0));
            null_vel_sample.setDistToPrefVel(Vector2.absSqr(pref_vel));
            samples.add(null_vel_sample);

            //sort according to the distance to vel pref
            if (samples.size() > 1)
                Collections.sort(samples, new Comparators.VelocitySamplesComparator());

            Vector2 new_vel = new Vector2();

            boolean valid;
            boolean foundOutside = false;
            boolean outside;
            int optimal = -1;

            for (int i = 0; i < samples.size(); i++) {
                outside = true;
                valid = true;
                if (!isWithinAdditionalConstraints(additional_constraints, samples.get(i).getVelocity())) {
                    outside = false;
                }

                for (int j = 0; j < truncated_vos.size(); j++) {
                    if (isInsideVO(truncated_vos.get(j), samples.get(i).getVelocity(), use_truncation)) {
                        valid = false;
                        if (j > optimal) {
                            optimal = j;
                            new_vel = samples.get(i).getVelocity();
                        }
                        break;
                    }
                }
                if (valid && outside) {
                    return samples.get(i).getVelocity();
                }
                //unless find one that satisfy both, otherwise return the one satisfy all truncated vos
                if (valid && !outside && !foundOutside) {
                    optimal = truncated_vos.size();
                    new_vel = samples.get(i).getVelocity();
                    foundOutside = true;
                }

            }

            //make sure to satisfy truncated vos
            return new_vel;
        }

        static boolean isWithinAdditionalConstraints(final List<Line> additional_constraints, final Vector2 point) {
            for (int i = 0; i < additional_constraints.size(); i++) {
                if (rightOf(additional_constraints.get(i).getPoint(), additional_constraints.get(i).getDir(), point)) {
                    return false;
                }
            }
            return true;
        }


        static boolean isInsideVO(final VO vo, Vector2 point, boolean use_truncation) {
            boolean trunc = leftOf(vo.getTruncLeft(), Vector2.minus(vo.getTruncRight(), vo.getTruncLeft()), point);
            if (Vector2.abs(Vector2.minus(vo.getTruncLeft(), vo.getTruncRight())) < Parameters.EPSILON)
                trunc = true;
            return rightOf(vo.getPoint(), vo.getLeftLegDir(), point) && leftOf(vo.getPoint(), vo.getRightLegDir(), point) && (!use_truncation || trunc);
        }

        static void addRayVelocitySamples(
                List<VelocitySample> samples,
                final Vector2 pref_vel,
                final Vector2 point1,
                final Vector2 dir1,
                final Vector2 point2,
                final Vector2 dir2, double max_speed, int TYPE) {

            double r, s;
            double x1, x2, x3, x4, y1, y2, y3, y4;
            x1 = point1.getX();
            y1 = point1.getY();
            x2 = x1 + dir1.getX();
            y2 = y1 + dir1.getY();
            x3 = point2.getX();
            y3 = point2.getY();
            x4 = x3 + dir2.getX();
            y4 = y3 + dir2.getY();

            double det = (((x2 - x1) * (y4 - y3)) - (y2 - y1) * (x4 - x3));

            if (det == 0.0) {
                return;
            }
            if (det != 0) {
                r = (((y1 - y3) * (x4 - x3)) - (x1 - x3) * (y4 - y3)) / det;//distance to point1????
                s = (((y1 - y3) * (x2 - x1)) - (x1 - x3) * (y2 - y1)) / det;

                if ((TYPE == LINELINE) || (TYPE == RAYLINE && r >= 0) || (TYPE == SEGMENTLINE && r >= 0 && r <= 1) || (TYPE == RAYRAY && r >= 0 && s >= 0) ||
                        (TYPE == RAYSEGMENT && r >= 0 && s >= 0 && s <= 1) || (TYPE == SEGMENTSEGMENT && r >= 0 && s >= 0 && r <= 1 && s <= 1)) {
                    VelocitySample intersection_point = new VelocitySample();
                    intersection_point.setVelocity(new Vector2(x1 + r * (x2 - x1), y1 + r * (y2 - y1)));
                    intersection_point.setDistToPrefVel(Vector2.absSqr(Vector2.minus(pref_vel, intersection_point.getVelocity())));
                    if (Vector2.absSqr(intersection_point.getVelocity()) < Math.pow(1.2 * max_speed, 2)) {
                        samples.add(intersection_point);
                    }
                }
            }
        }

        public static VO createObstacleVO(
                Vector2 position1,
                double radius1,
                final List<Vector2> footprint1,
                Vector2 obst1,
                Vector2 obst2) {

            VO result = new VO();
            //obstacle mid point
            Vector2 position_obst = Vector2.scale(Vector2.plus(obst1, obst2), 0.5);

            List<Vector2> obst = new ArrayList<Vector2>();
            obst.add(Vector2.minus(obst1, position_obst));
            obst.add(Vector2.minus(obst2, position_obst));

            List<Vector2> mink_sum = minkowskiSumConvexHull(footprint1, obst);

            Vector2 min_left, min_right;
            double min_ang = 0.0;
            double max_ang = 0.0;
            Vector2 rel_position = Vector2.minus(position_obst, position1);

            Vector2 rel_position_normal = Vector2.normal(rel_position);
            double min_dist = Vector2.abs(rel_position);

            for (int i = 0; i < mink_sum.size(); i++) {
                double angle = Vector2.angleBetween(rel_position, Vector2.plus(rel_position, mink_sum.get(i)));
                if (rightOf(new Vector2(0.0, 0.0), rel_position, Vector2.plus(rel_position, mink_sum.get(i)))) {
                    if (-angle < min_ang) {
                        min_right = Vector2.plus(rel_position, mink_sum.get(i));
                        min_ang = -angle;
                    }
                } else {
                    if (angle > max_ang) {
                        min_left = Vector2.plus(rel_position, mink_sum.get(i));
                        max_ang = angle;
                    }
                }
                Vector2 project_on_rel_position = intersectTwoLines(new Vector2(0.0, 0.0), rel_position, Vector2.plus(rel_position, mink_sum.get(i)), rel_position_normal);
                double dist = Vector2.abs(project_on_rel_position);
                if (Vector2.dotProduct(project_on_rel_position, rel_position) < -Parameters.EPSILON) {
                    dist = -dist;
                }

                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
            if (min_dist < 0) {
                // the left side of left leg and right side of right leg is the collision free area
                result.setRelativePosition(rel_position);
                result.setLeftLegDir(Vector2.negative(Vector2.normalize(Vector2.minus(obst1, obst2))));//?????????????
                result.setRightLegDir(Vector2.negative(result.getLeftLegDir()));

                result.setPoint(Vector2.minus(rel_position, Vector2.scale(Vector2.normal(result.getLeftLegDir()), 1.5 * radius1)));
                result.setTruncLeft(result.getPoint());
                result.setTruncRight(result.getPoint());
                return result;

            }

            double ang_rel = Math.atan2(rel_position.getY(), rel_position.getX());
            result.setLeftLegDir(new Vector2(Math.cos(ang_rel + max_ang), Math.sin(ang_rel + max_ang)));
            result.setRightLegDir(new Vector2(Math.cos(ang_rel + min_ang), Math.sin(ang_rel + min_ang)));
            // set margin
            result.setLeftLegDir(Vector2.rotateVectorByAngle(result.getLeftLegDir(), 0.05));
            result.setRightLegDir(Vector2.rotateVectorByAngle(result.getRightLegDir(), -0.05));

            result.setRelativePosition(rel_position);
            result.setCombinedRadius(Vector2.abs(rel_position) - min_dist);
            result.setPoint(new Vector2(0, 0));

            result.setTruncLeft(intersectTwoLines(result.getPoint(), result.getLeftLegDir(), Vector2.scale(Vector2.normalize(rel_position), result.getCombinedRadius() / 2.0), Vector2.minus(obst2, obst1)));
            result.setTruncRight(intersectTwoLines(result.getPoint(), result.getRightLegDir(), Vector2.scale(Vector2.normalize(rel_position), result.getCombinedRadius() / 2.0), Vector2.minus(obst2, obst1)));

            return result;

        }

        // intersection point of two lines
        static Vector2 intersectTwoLines(Vector2 point1, Vector2 dir1, Vector2 point2, Vector2 dir2) {
            double x1, x2, x3, x4, y1, y2, y3, y4;
            x1 = point1.getX();
            y1 = point1.getY();
            x2 = x1 + dir1.getX();
            y2 = y1 + dir1.getY();
            x3 = point2.getX();
            y3 = point2.getY();
            x4 = x3 + dir2.getX();
            y4 = y3 + dir2.getY();

            double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

            if (det == 0) {
                return new Vector2(0, 0); //TODO fix return NULL

            }
            double x_i = ((x3 - x4) * (x1 * y2 - y1 * x2) - (x1 - x2) * (x3 * y4 - y3 * x4)) / det;
            double y_i = ((y3 - y4) * (x1 * y2 - y1 * x2) - (y1 - y2) * (x3 * y4 - y3 * x4)) / det;

            return new Vector2(x_i, y_i);
        }

        // dissertation 2.2.2
        //by footprint
        public static VO createVO(Vector2 position1, final List<Vector2> footprint1, Vector2 vel1, Vector2 position2, final List<Vector2> footprint2, Vector2 vel2, int TYPE) {
            if (TYPE == HRVOS) {
                return createHRVO(position1, footprint1, vel1, position2, footprint2, vel2);
            } else if (TYPE == RVOS) {
                return createRVO(position1, footprint1, vel1, position2, footprint2, vel2);
            } else {
                return createVO(position1, footprint1, position2, footprint2, vel2);
            }
        }

        //by radius
        public static VO createVO(Vector2 position1, double radius1, Vector2 vel1, Vector2 position2, double radius2, Vector2 vel2, int TYPE) {
            if (TYPE == HRVOS) {
                return createHRVO(position1, radius1, vel1, position2, radius2, vel2);
            } else if (TYPE == RVOS) {
                return createRVO(position1, radius1, vel1, position2, radius2, vel2);
            } else {
                return createVO(position1, radius1, position2, radius2, vel2);
            }
        }

        //different types
        //by footprint
        private static VO createHRVO(Vector2 position1, final List<Vector2> footprint1, Vector2 vel1, Vector2 position2, final List<Vector2> footprint2, Vector2 vel2) {

            VO result = createRVO(position1, footprint1, vel1, position2, footprint2, vel2);
            Vector2 rel_velocity = Vector2.minus(vel1, vel2);

            if (result.getCombinedRadius() > Vector2.abs(result.getRelativePosition())) {
                return result;
            }

            if (leftOf(new Vector2(0.0, 0.0), result.getRelativePosition(), rel_velocity)) { //left of centerline
                result.setPoint(intersectTwoLines(result.getPoint(), result.getLeftLegDir(), vel2, result.getRightLegDir())); // TODO add uncertainty
            } else { //right of centerline
                result.setPoint(intersectTwoLines(vel2, result.getLeftLegDir(), result.getPoint(), result.getRightLegDir())); // TODO add uncertainty
            }
            return result;
        }

        //by radius
        private static VO createHRVO(Vector2 position1, double radius1, Vector2 vel1, Vector2 position2, double radius2, Vector2 vel2) {

            VO result = createRVO(position1, radius1, vel1, position2, radius2, vel2);

            Vector2 rel_velocity = Vector2.minus(vel1, vel2);
            Vector2 rel_position = Vector2.minus(position2, position1);
            if (Vector2.abs(rel_position) < radius1 + radius2) {
                result.setPoint(Vector2.scale(Vector2.plus(vel2, vel1), 0.5));
                return result;
            }

            if (leftOf(new Vector2(0.0, 0.0), rel_position, rel_velocity)) { //left of centerline
                result.setPoint(intersectTwoLines(result.getPoint(), result.getLeftLegDir(), vel2, result.getRightLegDir())); // TODO add uncertainty
            } else { //right of centerline
                result.setPoint(intersectTwoLines(vel2, result.getLeftLegDir(), result.getPoint(), result.getRightLegDir())); // TODO add uncertainty
            }
            return result;

        }

        //by footprint
        private static VO createRVO(Vector2 position1, final List<Vector2> footprint1, Vector2 vel1, Vector2 position2, final List<Vector2> footprint2, Vector2 vel2) {
            VO result = createVO(position1, footprint1, position2, footprint2, vel2);
            result.setPoint(Vector2.scale(Vector2.plus(vel1, vel2), 0.5)); //TODO add uncertainty
            return result;
        }

        //by radius
        private static VO createRVO(Vector2 position1, double radius1, Vector2 vel1, Vector2 position2, double radius2, Vector2 vel2) {
            VO result = createVO(position1, radius1, position2, radius2, vel2);
            result.setPoint(Vector2.scale(Vector2.plus(vel1, vel2), 0.5)); //TODO add uncertainty
            return result;
        }

        //basic method by footprint
        private static VO createVO(Vector2 position1, List<Vector2> footprint1, Vector2 position2, List<Vector2> footprint2, Vector2 vel2) {
            VO result = new VO();
            List<Vector2> mink_sum = minkowskiSumConvexHull(footprint1, footprint2);

            Vector2 min_left = new Vector2();
            Vector2 min_right = new Vector2();
            double min_ang = 0.0;
            double max_ang = 0.0;
            Vector2 rel_position = Vector2.minus(position2, position1);

            Vector2 rel_position_normal = Vector2.normal(rel_position);
            double min_dist = Vector2.abs(rel_position);

            for (int i = 0; i < mink_sum.size(); i++) {
                double angle = Vector2.angleBetween(rel_position, Vector2.plus(rel_position, mink_sum.get(i)));
                if (rightOf(new Vector2(0.0, 0.0), rel_position, Vector2.plus(rel_position, mink_sum.get(i)))) {
                    if (-angle < min_ang) {
                        min_right = Vector2.plus(rel_position, mink_sum.get(i));
                        min_ang = -angle;
                    }
                } else {
                    if (angle > max_ang) {
                        min_left = Vector2.plus(rel_position, mink_sum.get(i));
                        max_ang = angle;
                    }
                }
                Vector2 project_on_rel_position = intersectTwoLines(new Vector2(0.0, 0.0), rel_position, Vector2.plus(rel_position, mink_sum.get(i)), rel_position_normal);
                double dist = Vector2.abs(project_on_rel_position);
                if (Vector2.dotProduct(project_on_rel_position, rel_position) < -Parameters.EPSILON) {
                    dist = -dist;

                }

                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
            if (min_dist < 0) {
                result.setLeftLegDir(Vector2.negative(Vector2.normalize(rel_position_normal)));
                result.setRightLegDir(Vector2.negative(result.getLeftLegDir()));
                result.setRelativePosition(rel_position);
                result.setCombinedRadius(Vector2.abs(rel_position) - min_dist);
                result.setPoint(vel2);
                return result;
            }

            double ang_rel = Math.atan2(rel_position.getY(), rel_position.getX());
            result.setLeftLegDir(new Vector2(Math.cos(ang_rel + max_ang), Math.sin(ang_rel + max_ang)));
            result.setRightLegDir(new Vector2(Math.cos(ang_rel + min_ang), Math.sin(ang_rel + min_ang)));

            result.setLeftLegDir(Vector2.rotateVectorByAngle(result.getLeftLegDir(), 0.15));
            result.setRightLegDir(Vector2.rotateVectorByAngle(result.getRightLegDir(), -0.05));


            double ang_between = Vector2.angleBetween(result.getRightLegDir(), result.getLeftLegDir());
            double opening_ang = ang_rel + min_ang + (ang_between) / 2.0;

            Vector2 dir_center = new Vector2(Math.cos(opening_ang), Math.sin(opening_ang));
            min_dist = Vector2.abs(rel_position);
            Vector2 min_point = new Vector2(rel_position);
            for (int i = 0; i < mink_sum.size(); i++) {
                Vector2 proj_on_center = intersectTwoLines(new Vector2(0.0, 0.0), dir_center, Vector2.plus(rel_position, mink_sum.get(i)), Vector2.normal(dir_center));
                if (Vector2.abs(proj_on_center) < min_dist) {
                    min_dist = Vector2.abs(proj_on_center);
                    min_point = Vector2.plus(rel_position, mink_sum.get(i));
                }

            }

            double center_p, radius;
            Vector2 center_r;
            //test left/right
            if (Vector2.abs(min_left) < Vector2.abs(min_right)) {
                center_p = Vector2.abs(min_left) / Math.cos(ang_between / 2.0);
                radius = Math.tan(ang_between / 2.0) * Vector2.abs(min_left);
                center_r = Vector2.scale(dir_center, center_p);
            } else {
                center_p = Vector2.abs(min_right) / Math.cos(ang_between / 2.0);
                center_r = Vector2.scale(dir_center, center_p);
                radius = Math.tan(ang_between / 2.0) * Vector2.abs(min_right);
            }
            //check min_point, if failed stupid calc for new radius and point;
            if (Vector2.abs(Vector2.minus(min_point, center_r)) > radius) {
                double gamma = min_point.getX() * dir_center.getX() + min_point.getY() * dir_center.getY();
                double sqrt_exp = Vector2.absSqr(min_point) / (Math.pow(Math.sin(ang_between / 2.0), 2) - 1) + Math.pow(gamma / (Math.pow(Math.sin(ang_between / 2.0), 2) - 1), 2);
                if (Math.abs(sqrt_exp) < Parameters.EPSILON) {
                    sqrt_exp = 0;
                }
                if (sqrt_exp >= 0) {
                    center_p = -gamma / (Math.pow(Math.sin(ang_between / 2.0), 2) - 1) + Math.sqrt(sqrt_exp);
                    center_r = Vector2.scale(dir_center, center_p);
                    radius = Vector2.abs(Vector2.minus(min_point, center_r));
                } else {
                    logger.warn("CreateVO warn: ang =" + ang_between + ", sqrt_ext = " + sqrt_exp);
                    logger.warn("CreateVO warn: rel position " + rel_position.getX() + " " + rel_position.getY() + ", radius " + radius + ", center_p " + center_p);
                    logger.warn("Agent position: {}, neighbor position: {}", position1.toString(), position2.toString());
                }
            }

            result.setRelativePosition(center_r);
            result.setCombinedRadius(radius);
            result.setPoint(vel2);
            return result;

        }

        //basic method by radius
        private static VO createVO(Vector2 position1, double radius1, Vector2 position2, double radius2, Vector2 vel2) {
            VO result = new VO();
            Vector2 rel_position = Vector2.minus(position2, position1);
            double ang_to_other = Vector2.atan(rel_position);
            double combined_radius = radius2 + radius1;
            double angle_of_opening;
            if (Vector2.abs(rel_position) < combined_radius) {

                result.setLeftLegDir(Vector2.negative(Vector2.normalize(Vector2.normal(rel_position))));
                result.setRightLegDir(Vector2.negative(result.getLeftLegDir()));
            } else {
                angle_of_opening = Math.asin(combined_radius / Vector2.abs(rel_position));
                result.setRightLegDir(new Vector2(Math.cos(ang_to_other - angle_of_opening), Math.sin(ang_to_other - angle_of_opening)));
                result.setLeftLegDir(new Vector2(Math.cos(ang_to_other + angle_of_opening), Math.sin(ang_to_other + angle_of_opening)));
            }
            result.setPoint(vel2);
            result.setRelativePosition(rel_position);
            result.setCombinedRadius(radius1 + radius2);

            return result;
        }


        public static VO createTruncVO(VO vo, double time) {
            VO result = new VO();
            result.setPoint(vo.getPoint());
            result.setLeftLegDir(vo.getLeftLegDir());
            result.setRightLegDir(vo.getRightLegDir());
            result.setRelativePosition(vo.getRelativePosition());
            result.setCombinedRadius(vo.getCombinedRadius());
            double trunc_radius = vo.getCombinedRadius() / time;
            double angle_of_opening;

            if (Vector2.abs(vo.getRelativePosition()) < vo.getCombinedRadius()) {
                result.setTruncLeft(result.getPoint());
                result.setTruncRight(result.getPoint());
                result.setTruncLineCenter(result.getPoint());
                return result;
            } else {
                angle_of_opening = Math.asin(vo.getCombinedRadius() / Vector2.abs(vo.getRelativePosition()));
                double trunc_dist = trunc_radius / Math.sin(angle_of_opening) - trunc_radius;
                result.setTruncLineCenter(Vector2.scale(Vector2.normalize(vo.getRelativePosition()), trunc_dist));
                Vector2 intersectLeft = intersectTwoLines(Vector2.plus(result.getPoint(), result.getTruncLineCenter()),
                        new Vector2(result.getTruncLineCenter().getY(), -result.getTruncLineCenter().getX()), result.getPoint(), result.getLeftLegDir());
                result.setTruncLeft(intersectLeft);
                result.setTruncRight(intersectTwoLines(Vector2.plus(result.getPoint(), result.getTruncLineCenter()),
                        new Vector2(result.getTruncLineCenter().getY(), -result.getTruncLineCenter().getX()), result.getPoint(), result.getRightLegDir()));

                return result;
            }
        }

        //-------------------compute obstacle vos-------------------------
        public static void computeObstacleVOs(Agent agent) {
            if (agent.obstacles_from_laser_.size() <= 0)
                return;
            double maxDist = Math.pow((Vector2.abs(agent.velocity) + 4.0 * agent.footprint_radius_), 2);
            for (Obstacle obstacle : agent.obstacles_from_laser_) {
                //why choose this limit?????????????????????????????????
                if (agent.use_obstacles_) {
                    if (obstacle.getDistToAgent() < maxDist) {
                        if (agent.orca) {// currently set to false
                            createObstacleLine(agent, obstacle.getBegin(), obstacle.getEnd());
                        } else {//called from CP
                            VO obstacle_vo = ClearPath.createObstacleVO(
                                    agent.position.getPos(),
                                    agent.footprint_radius_,
                                    agent.footprint_original,
                                    obstacle.getBegin(),
                                    obstacle.getEnd()
                            );
                            obstacle_vo.setType("obstacleVo");
                            agent.voAgents.add(obstacle_vo);
                        }
                    }
                }

            }
        }

        static void createObstacleLine(Agent agent, Vector2 obst1, Vector2 obst2) {

            double dist = distSqPointLineSegment(obst1, obst2, agent.position.getPos());

            if (dist == Vector2.absSqr(Vector2.minus(agent.position.getPos(), obst1))) {
                computeObstacleLine(agent, obst1);
            } else if (dist == Vector2.absSqr(Vector2.minus(agent.position.getPos(), obst2))) {
                computeObstacleLine(agent, obst2);
            } else {
                Vector2 position_obst = projectPointOnLine(obst1, Vector2.minus(obst2, obst1), agent.position.getPos());
                Vector2 rel_position = Vector2.minus(position_obst, agent.position.getPos());
                dist = Math.sqrt(dist);
                double dist_to_footprint = getDistToFootprint(agent, rel_position);
                if (dist_to_footprint == -1) {
                    dist_to_footprint = agent.footprint_radius_;
                }
                dist = dist - dist_to_footprint - 0.03;

                if (dist < 0.0) {
                    Line line = new Line();
                    line.setPoint(Vector2.scale(Vector2.normalize(rel_position), (dist - 0.02)));
                    line.setDir(Vector2.normalize(Vector2.minus(obst1, obst2)));
                    agent.addOrcaLines.add(line);
                    return;
                }

                if (Vector2.abs(Vector2.minus(agent.position.getPos(), obst1)) > 2 * agent.footprint_radius_ &&
                        Vector2.abs(Vector2.minus(agent.position.getPos(), obst2)) > 2 * agent.footprint_radius_) {
                    Line line = new Line();
                    line.setPoint(Vector2.scale(Vector2.normalize(rel_position), dist));
                    line.setDir(Vector2.negative(Vector2.normalize(Vector2.minus(obst1, obst2))));
                    agent.addOrcaLines.add(line);
                    return;

                }

                rel_position = Vector2.scale(Vector2.normalize(rel_position), Vector2.abs(rel_position) - dist / 2.0);

                List<Vector2> obst = new ArrayList<Vector2>();
                obst.add(Vector2.minus(obst1, position_obst));
                obst.add(Vector2.minus(obst2, position_obst));
                List<Vector2> mink_sum = minkowskiSumConvexHull(agent.footprint_original, obst);

                Vector2 min = new Vector2();
                Vector2 max = new Vector2();
                double min_ang = 0.0;
                double max_ang = 0.0;

                for (int i = 0; i < mink_sum.size(); i++) {
                    double angle = Vector2.angleBetween(rel_position, Vector2.plus(rel_position, mink_sum.get(i)));
                    if (leftOf(new Vector2(0.0, 0.0), rel_position, Vector2.plus(rel_position, mink_sum.get(i)))) {
                        if (-angle < min_ang) {
                            min = Vector2.plus(rel_position, mink_sum.get(i));
                            min_ang = -angle;
                        }
                    } else {
                        if (angle > max_ang) {
                            max = Vector2.plus(rel_position, mink_sum.get(i));
                            max_ang = angle;
                        }
                    }
                }

                Line line = new Line();
                line.setPoint(Vector2.scale(Vector2.normalize(rel_position), dist / 2.0));
                if (Vector2.absSqr(Vector2.minus(position_obst, obst1)) > Vector2.absSqr(Vector2.minus(position_obst, obst2))) {
                    line.setDir(Vector2.rotateVectorByAngle(Vector2.normalize(max), 0.1));
                } else {
                    line.setDir(Vector2.rotateVectorByAngle(Vector2.normalize(min), 0.1));

                }
                agent.addOrcaLines.add(line);

            }
        }

        static void computeObstacleLine(Agent agent, Vector2 obst) {
            Line line = new Line();
            Vector2 relative_position = Vector2.minus(obst, agent.position.getPos());
            double dist_to_footprint;
            double dist = Vector2.abs(Vector2.minus(agent.position.getPos(), obst));
            if (!agent.has_polygon_footprint_)
                dist_to_footprint = agent.footprint_radius_;
            else {
                dist_to_footprint = getDistToFootprint(agent, relative_position);
                if (dist_to_footprint == -1) {
                    dist_to_footprint = agent.footprint_radius_;
                }
            }
            dist = dist - dist_to_footprint - 0.03;

            line.setPoint(Vector2.scale(Vector2.normalize(relative_position), dist));
            line.setDir(new Vector2(-(Vector2.normalize(relative_position)).getY(),
                    (Vector2.normalize(relative_position)).getX()));
            agent.addOrcaLines.add(line);
        }

        static double getDistToFootprint(Agent agent, Vector2 point) {
            Vector2 result;
            for (int i = 0; i < agent.footprint_minkowski_lines.size(); i++) {
                LinePair l1 = new LinePair(agent.footprint_minkowski_lines.get(i).getFirst(),
                        agent.footprint_minkowski_lines.get(i).getSecond());
                LinePair l2 = new LinePair(new Vector2(0.0, 0.0), point);

                result = LinePair.Intersection(l1, l2);
                if (result != null) {
                    return Vector2.abs(result);
                }
            }
            return -1;
        }

        //--------------------------agent vos-----------------------------

        public static void computeAgentVOs(Agent agent) {
            double radiusWithError = agent.radius + agent.cur_allowed_nh_error_;
            // neighbors are published with localization uncertainty that means radius and footprint
            // include localization uncertainty radius and minkowski footprint.
            for (Neighbor neighbor : agent.AgentNeighbors) {
                VO new_agent_vo;
                //use footprint or radius to create VO
                if (agent.convex) {
                    if (neighbor.controlled) {
                        new_agent_vo = ClearPath.createVO(
                                agent.position.getPos(),
                                agent.footprint_rotated,
                                agent.velocity,
                                neighbor.position.getPos(),
                                neighbor.footprint_rotated,
                                neighbor.velocity,
                                agent.voType);
                    } else {
                        new_agent_vo = ClearPath.createVO(
                                agent.position.getPos(),
                                agent.footprint_rotated,
                                agent.velocity,
                                neighbor.position.getPos(),
                                neighbor.footprint_rotated,
                                neighbor.velocity,
                                ClearPath.VOS);
                    }
                } else {
                    if (neighbor.controlled) {
                        new_agent_vo = ClearPath.createVO(
                                agent.position.getPos(),
                                radiusWithError,
                                agent.velocity,
                                neighbor.position.getPos(),
                                neighbor.radius,
                                neighbor.velocity,
                                agent.voType);
                    } else {
                        new_agent_vo = ClearPath.createVO(
                                agent.position.getPos(),
                                radiusWithError,
                                agent.velocity,
                                neighbor.position.getPos(),
                                neighbor.radius,
                                neighbor.velocity,
                                ClearPath.VOS);
                    }
                }
                //truncation--not collide in certain amount of time
                if (agent.useTruancation) {
                    new_agent_vo = ClearPath.createTruncVO(new_agent_vo, agent.truncTime);
                    new_agent_vo.setType("agentVo");
                }
                agent.voAgents.add(new_agent_vo);
            }
        }

    }


    public static void velocityToCmd(Agent agent) {
        double speed_ang = Math.atan2(agent.newVelocity.getY(), agent.newVelocity.getX());
        double dif_ang = shortest_angular_distance(agent.position.getHeading(), speed_ang);

        Vector3d_ linear = new Vector3d_();
        Vector3d_ angular = new Vector3d_();
        if (!agent.holo_robot_) {
            double vel = Vector2.abs(agent.newVelocity);
            double vstar;

            if (Math.abs(dif_ang) > Parameters.EPSILON)
                vstar = NHORCA.calcVstar(vel, dif_ang);//get nonholomonic velocity
            else
                vstar = agent.max_vel_x_;

            linear.setX(Math.min(vstar, NHORCA.vMaxAng(agent)));
            linear.setY(0.0);
            agent.cmd_vel.setLinear(linear);

            if (Math.abs(dif_ang) > 3.0 * Math.PI / 4.0) {
                angular.setZ(
                        sign(
                                agent.base_odom_.getTwist().getAngular().getZ()) *
                                Math.min(Math.abs(dif_ang / agent.time_to_holo_),
                                        agent.max_vel_th_)
                );
                agent.cmd_vel.setAngular(angular);
            } else {
                angular.setZ(
                        sign(dif_ang) *
                                Math.min(Math.abs(dif_ang / agent.time_to_holo_),
                                        agent.max_vel_th_)
                );
                agent.cmd_vel.setAngular(angular);
            }
        } else {
            //new velocity is caculated in world frame, it has to be transformed to robot base frame
            Vector2 rotated_vel = Vector2.rotateVectorByAngle(agent.newVelocity, -agent.position.getHeading());
            linear.setX(rotated_vel.getX());
            linear.setY(rotated_vel.getY());
            agent.cmd_vel.setLinear(linear);
            if (agent.min_dist > 2 * agent.footprint_radius_) {
                angular.setZ(sign(dif_ang) * Math.min(Math.abs(dif_ang), agent.max_vel_th_));
                agent.cmd_vel.setAngular(angular);
            }
        }

    }


    public static void getValidCommand(Agent agent) {
        if (Math.abs(agent.cmd_vel.getAngular().getZ()) > agent.max_vel_th_)
            agent.cmd_vel.getAngular().setZ(Methods_Planners.sign(agent.cmd_vel.getAngular().getZ()) * agent.max_vel_th_);

        if (Math.abs(agent.cmd_vel.getLinear().getX()) > agent.max_vel_x_)
            agent.cmd_vel.getLinear().setX(Methods_Planners.sign(agent.cmd_vel.getLinear().getX()) * agent.max_vel_x_);

        if (Math.abs(agent.cmd_vel.getLinear().getY()) > agent.max_vel_y_)
            agent.cmd_vel.getLinear().setY(Methods_Planners.sign(agent.cmd_vel.getLinear().getY()) * agent.max_vel_y_);

        if (Math.abs(agent.cmd_vel.getAngular().getZ()) < agent.min_vel_th_)
            agent.cmd_vel.getAngular().setZ(0);

        if (Math.abs(agent.cmd_vel.getLinear().getX()) < agent.min_vel_x_)
            agent.cmd_vel.getLinear().setX(0);

        if (Math.abs(agent.cmd_vel.getLinear().getY()) < agent.min_vel_y_)
            agent.cmd_vel.getLinear().setY(0);
    }

    // global planner
    public static List<PoseStamped_> makePlan(final PoseStamped_ start, final PoseStamped_ goal) {
        //start and goal are supposed not to change
        List<PoseStamped_> plan = new ArrayList<PoseStamped_>();
        plan.clear();
        plan.add(start);
        double x, y, dir_x, dir_y;
        dir_x = goal.getPose().getPosition().getX() - start.getPose().getPosition().getX();
        dir_y = goal.getPose().getPosition().getY() - start.getPose().getPosition().getY();
        double length = Math.sqrt(dir_y * dir_y + dir_x * dir_x);
        dir_x /= length;
        dir_y /= length;
        x = start.getPose().getPosition().getX() + 0.1 * dir_x;
        y = start.getPose().getPosition().getY() + 0.1 * dir_y;

        while (Math.abs(x - goal.getPose().getPosition().getX()) > 0.2 || Math.abs(y - goal.getPose().getPosition().getY()) > 0.2) {
            PoseStamped_ pose = new PoseStamped_();
            pose.getPose().getPosition().setX(x);
            pose.getPose().getPosition().setY(y);
            pose.getHeader().setFrameId(goal.getHeader().getFrameId());
            pose.getHeader().setStamp(goal.getHeader().getStamp());
            plan.add(pose);
            x += 0.1 * dir_x;
            y += 0.1 * dir_y;
        }
        plan.add(goal);
        return plan;
    }

}
