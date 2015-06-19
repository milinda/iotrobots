package cgl.iotrobots.collavoid.topology;

/* Some of the methods are based on multi_robot_collision_avoidance on wiki ros, license information: refer to
 http://wiki.ros.org/action/login/multi_robot_collision_avoidance */

import cgl.iotrobots.collavoid.commons.planners.*;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Methods {
    private static Logger logger = LoggerFactory.getLogger(Methods.class);

    public static boolean isCloseNeighbor(Odometry_ odom, double radius, PoseShareMsg_ msg, double trunctime) {
        double truncdist;
        double vel = odom.getTwist().getLinear().length();

        truncdist = trunctime * vel + 6 * radius;

        if (Methods_Planners.getPositionDistance(odom.getPose(), msg.getPose())
                < truncdist)
            return true;
        else
            return false;

    }

    public static PoseShareMsg_ extractPoseShareMsgFromAgent(Agent agent) {
        PoseShareMsg_ msg = new PoseShareMsg_();
        msg.setId(agent.id);
        msg.setRadius(agent.radius);
        msg.setHoloRobot(agent.getHoloRobot());
        msg.setControlled(agent.getControlled());
        msg.setPulishMePeriod(agent.publishMePeriod);

        List<Vector2> footprint = new ArrayList<Vector2>();
        for (Vector2 pt : agent.footprint_original) {
            footprint.add(new Vector2(pt.getX(), pt.getY()));
        }
        msg.setFootprint_original(footprint);
        msg.getHeader().setSeq(agent.getSeq());
        return msg;
    }

    public static Neighbor extractNeighbor(PoseShareMsg_ msg) {
        Neighbor neighbor = new Neighbor(msg.getId());
        neighbor.holo_robot_ = msg.getHoloRobot();
        neighbor.getBaseOdom().setPose(msg.getPose());
        neighbor.getBaseOdom().setTwist(msg.getTwist());
        neighbor.radius = msg.getRadius();
        neighbor.controlled = msg.getControlled();
        neighbor.setControlPeriod(msg.getPulishMePeriod());
        neighbor.setFootprint_minkowski(msg.getFootPrint_Minkowski());
        neighbor.last_seen_ = msg.getHeader().getStamp();

        return neighbor;
    }


    public static List<Obstacle> getObstacles(List<Neighbor> neighbors, double footprint_radius_, PointCloud2_ msg) {
        //in global frame
        List<Obstacle> obstacles = new ArrayList<Obstacle>();

        if (msg.getWidth() * msg.getHeight() == 0) {
            return obstacles;
        }
        if (msg.getData().length % 3 != 0) {
            return obstacles;
        }
        List<Vector3d_> point3ds = new ArrayList<Vector3d_>();

        double[] data = msg.getData();
        int pn = data.length / 3;
        int idx = 0;
        int idx_;
        while (idx < pn) {
            idx_ = idx * 3;
            point3ds.add(new Vector3d_(data[idx_], data[idx_ + 1], data[idx_ + 2]));
            idx++;
        }

        double threshold_convex = 0.03;
        double threshold_concave = -0.03;
        Vector2 start;
        for (int i = 0; i < point3ds.size(); i++) {
            start = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
            while (pointInNeighbor(neighbors, start) && i < point3ds.size() - 1) {
                i++;
                start = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
            }
            if (i == point3ds.size()) {
                //it is a agent
                return obstacles;
            }

            boolean found = false;
            Vector2 prev = new Vector2(start);
            double first_ang = 0;
            double prev_ang = 0;
            Vector2 next;
            while (!found) {
                i++;
                if (i == point3ds.size()) {
                    break;
                }
                next = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
                while (pointInNeighbor(neighbors, next) && i < point3ds.size() - 1) {
                    i++;
                    next = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
                }

                if (Vector2.abs(Vector2.minus(next, prev)) > 2 * footprint_radius_) {
                    found = true;
                    break;
                }
                Vector2 dif = Vector2.minus(next, start);
                double ang = Math.atan2(dif.getY(), dif.getX());
                if (!prev.equals(start)) {
                    if (ang - first_ang < threshold_concave) {
                        found = true;
                        i -= 2;
                        break;
                    }
                    if (ang - prev_ang < threshold_concave) {
                        found = true;
                        i -= 2;
                        break;
                    }
                    if (ang - prev_ang > threshold_convex) { //going towards me
                        found = true;
                        i -= 2;
                        break;
                    }
                    if (ang - first_ang > threshold_convex) { //going towards me
                        found = true;
                        i -= 2;
                        break;
                    }

                } else {
                    first_ang = ang;
                }
                prev = new Vector2(next);
                prev_ang = ang;
            }
            if (!start.equals(prev)) {
                Obstacle obst = new Obstacle(start, prev);
                obstacles.add(obst);
            }
        }
        return obstacles;

    }

    private static boolean pointInNeighbor(List<Neighbor> neighbors, Vector2 point) {
        double dist;
        for (Neighbor neighbor : neighbors) {
            dist = Vector2.abs(Vector2.minus(point, neighbor.position.getPos()));
            if (dist <= neighbor.radius) {
                return true;
            }
        }
        return false;
    }


    // for test
    public static List<Vector2> getMinkowskiFootprint(List<Vector2> footprint_original, PoseArray_ poseArray_) {
        // in robot base frame do not need transform
        double x, y;
        List<Vector2> localization_footprint = new ArrayList<Vector2>();
        // select valid localization
        if (poseArray_ != null) {
            for (int i = 0; i < poseArray_.getPoses().size(); i++) {
                x = poseArray_.getPoses().get(i).getPosition().getX();
                y = poseArray_.getPoses().get(i).getPosition().getY();
                Vector2 p = new Vector2(x, y);
                if (p.VectorLength() > Parameters.LOCALIZATION_ERROR_MAX)
                    continue;
                localization_footprint.add(p);
            }
        }
        return Methods_Planners.minkowskiSumConvexHull(localization_footprint, footprint_original);
    }


    public static void getCommand(Agent agent) {
        computeVelocity(agent);
        if (agent.prefVelociy != null) {
            forwardAgents(agent);
            agent.addOrcaLines.clear();
            agent.voAgents.clear();

            Methods_Planners.NHORCA.addNHConstraints(agent);
            Methods_Planners.NHORCA.addAccelerationConstraintsXY(
                    agent.max_vel_x_,
                    agent.acc_lim_x_,
                    agent.max_vel_y_,
                    agent.acc_lim_y_,
                    agent.velocity,
                    agent.position.getHeading(),
                    agent.controlPeriod,
                    agent.holo_robot_,
                    agent.addOrcaLines
            );
            Methods_Planners.ClearPath.computeObstacleVOs(agent);
            Methods_Planners.ClearPath.computeAgentVOs(agent);

            agent.newVelocity = Methods_Planners.ClearPath.calculateClearpathVelocity(
                    null,
                    agent.voAgents,
                    agent.addOrcaLines,
                    agent.prefVelociy,
                    agent.max_speed_x_,
                    agent.useTruancation);

            Methods_Planners.velocityToCmd(agent);
            checkVelocityCommand(agent);
        } else {
            Methods_Planners.getValidCommand(agent);
        }
    }

    private static void forwardAgents(Agent agent) {
        updateAgentState(agent);
        updateAllNeighbors(agent);
        computeMinDistToAll(agent);
    }


    private static void updateAllNeighbors(Agent agent) {
        for (Neighbor agent1 : agent.AgentNeighbors) {
            updateAgentState(agent1);
        }
        Collections.sort(agent.AgentNeighbors, new Comparators.NeighborDistComparator(agent.position.getPos()));
    }

    private static void updateAgentState(Neighbor agt) {
        double time_dif;
        if (agt.getLastSeen() == 0) {
            time_dif = 0;
        } else {
            time_dif = System.currentTimeMillis() - agt.getLastSeen();
        }
        time_dif = time_dif / 1000.0;

        double yaw, x_dif, y_dif, th_dif, x, y, theta;
        Vector2 pt = new Vector2(0.0, 0.0);

        //update position
        yaw = Methods_Planners.getYaw(agt.getBaseOdom().getPose().getOrientation());
        th_dif = time_dif * agt.getBaseOdom().getTwist().getAngular().getZ();
        if (agt.getHoloRobot()) {
            x_dif = time_dif * agt.getBaseOdom().getTwist().getLinear().getX();
            y_dif = time_dif * agt.getBaseOdom().getTwist().getLinear().getY();
        } else {
            x_dif = time_dif * agt.getBaseOdom().getTwist().getLinear().getX() * Math.cos(yaw + th_dif / 2.0);
            y_dif = time_dif * agt.getBaseOdom().getTwist().getLinear().getX() * Math.sin(yaw + th_dif / 2.0);
        }
        theta = yaw + th_dif;
        x = agt.getBaseOdom().getPose().getPosition().getX() + x_dif;
        y = agt.getBaseOdom().getPose().getPosition().getY() + y_dif;
        agt.setPosition(new Position(x, y, theta));

        //minkowski footprint is in robot frame, in velocity space only need orientation.
        agt.setFootprint_rotated(Methods_Planners.rotateFootprint(
                agt.getFootprint_minkowski(),
                agt.getPosition().getHeading()));

        //update velocity
        if (agt.getHoloRobot()) {
            x = agt.getBaseOdom().getTwist().getLinear().getX();
            y = agt.getBaseOdom().getTwist().getLinear().getY();
            pt.setX(x);
            pt.setY(y);
            agt.setVelocity(Vector2.rotateVectorByAngle(pt, (yaw + th_dif)));
        } else {
            double dif_x, dif_y, dif_ang;
            dif_ang = agt.getControlPeriod() * agt.getBaseOdom().getTwist().getAngular().getZ();
            //in robot frame differential robot has only x velocity
            pt.setX(agt.getBaseOdom().getTwist().getLinear().getX() * Math.cos(dif_ang / 2.0));
            pt.setY(agt.getBaseOdom().getTwist().getLinear().getX() * Math.sin(dif_ang / 2.0));
            // in global frame, velocity need no translation
            agt.setVelocity(Vector2.rotateVectorByAngle(pt, (yaw + th_dif)));
        }
    }

    private static void computeMinDistToAll(Agent agent) {
        double min_dist_neigh = Double.MAX_VALUE;
        double min_dist_obstacle = Double.MAX_VALUE;
        //neighbors have already been sorted according to their dist to me
        if (agent.AgentNeighbors.size() > 0)
            min_dist_neigh = Vector2.abs(Vector2.minus(
                            agent.AgentNeighbors.get(0).position.getPos(),
                            agent.position.getPos())
            );

        double threshold = Math.pow((Vector2.abs(agent.velocity) + 4.0 * agent.footprint_radius_), 2);
        for (Obstacle obstacle : agent.obstacles_from_laser_) {
            double dist = Methods_Planners.distSqPointLineSegment(
                    obstacle.getBegin(),
                    obstacle.getEnd(),
                    agent.position.getPos());
            obstacle.setDistToAgent(dist);
            if (dist < threshold) {
                if (dist < min_dist_obstacle) {
                    min_dist_obstacle = dist;
                }
            }

        }
        agent.min_dist = Math.min(min_dist_neigh, min_dist_obstacle);
    }

    private static boolean computeVelocity(Agent agent) {
        agent.prefVelociy = null;
        //get position and velocity,pose in global frame, velocity is in base frame
        PoseStamped_ global_pose = new PoseStamped_();
        global_pose.getHeader().setFrameId(agent.global_frame_);
        global_pose.getHeader().setStamp(System.currentTimeMillis());

        //velocity is in base frame
        //pose is in odometry or map frame
        global_pose.setPose(agent.getBaseOdom().getPose());

        Pose_ goal_point;
        goal_point = agent.plan.get(agent.plan.size() - 1).getPose();
        double goal_x = goal_point.getPosition().getX();
        double goal_y = goal_point.getPosition().getY();
        double goal_th = Methods_Planners.getYaw(goal_point.getOrientation());

        //check to see if we've reached the goal position
        if (agent.xy_tolerance_latch_ ||
                (Methods_Planners.getGoalPositionDistance(
                        global_pose.getPose(),
                        goal_x,
                        goal_y) <= agent.xy_goal_tolerance)) {

            //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
            //just rotate in place
            if (agent.latch_xy_goal_tolerance)
                agent.xy_tolerance_latch_ = true;

            //check to see if the goal orientation has been reached
            double angle = Methods_Planners.getGoalOrientationAngleDifference(global_pose.getPose(), goal_th);

            //check to see if the goal orientation has been reached
            // in continuous testing such tolerance may accumulate to a large deviation to the original position
            if (Math.abs(angle) <= agent.yaw_goal_tolerance) {
                //set the velocity command to zero
                agent.cmd_vel.setGoalReached(true);
                agent.cmd_vel.setAngular(new Vector3d_());
                agent.cmd_vel.setLinear(new Vector3d_());
            } else {
                //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
                if (!agent.rotating_to_goal_ && !Methods_Planners.stopped(
                        agent.getBaseOdom(),
                        agent.rot_stopped_velocity,
                        agent.trans_stopped_velocity)) {
                    stopWithAccLimits(agent);
                }
                //if we're stopped... then we want to rotate to goal
                else {
                    //set this so that we know its OK to be moving
                    agent.rotating_to_goal_ = true;
                    rotateToGoal(agent, goal_th);
                }
            }

            //publish an empty plan because we've reached our goal position
            agent.transformed_plan.clear();
            return true;
        }

        Pose_ target_pose = new Pose_();

        if (!agent.skip_next) {
            if (!transformGlobalPlan(agent)) {
                logger.warn(agent.id
                        + ": Could not transform the global plan to the frame of the controller");
                return false;
            }
            findBestWaypoint(target_pose, agent);
        } else {
            target_pose = agent.transformed_plan.get(agent.current_waypoint).getPose();
        }

        Twist_ pref_vel_twist = new Twist_();

        pref_vel_twist.getLinear().setX(target_pose.getPosition().getX() - global_pose.getPose().getPosition().getX());
        pref_vel_twist.getLinear().setY(target_pose.getPosition().getY() - global_pose.getPose().getPosition().getY());

        agent.prefVelociy = new Vector2(pref_vel_twist.getLinear().getX(), pref_vel_twist.getLinear().getY());

        if (Vector2.abs(agent.prefVelociy) > agent.max_vel_x_) {
            agent.prefVelociy = Vector2.scale(Vector2.normalize(agent.prefVelociy), agent.max_vel_x_);
        } else if (Vector2.abs(agent.prefVelociy) < agent.max_vel_x_) {
            agent.prefVelociy = Vector2.scale(
                    Vector2.normalize(agent.prefVelociy),
                    agent.max_vel_x_ * 1.2);
        }
        return true;
    }

    private static boolean transformGlobalPlan(Agent agent) {
        agent.transformed_plan.clear();
        if (!(agent.plan.size() > 0)) {
            logger.error("Recieved plan with zero length");
            return false;
        }
        //currently global plan is in global frame do not need transform, robot pose is also in global frame
        int cur_waypoint = 0;

        //we'll keep points on the plan that are within the window that we're looking at
        double sq_dist = Double.MAX_VALUE;
        double dist;
        for (int i = 0; i < agent.plan.size(); i++) {
            dist = Methods_Planners.getGoalPositionDistance(
                    agent.getBaseOdom().getPose(),
                    agent.plan.get(i).getPose().getPosition().getX(),
                    agent.plan.get(i).getPose().getPosition().getY()
            );
            if (dist < Math.sqrt(sq_dist)) {
                sq_dist = dist * dist;
                cur_waypoint = i;
            }
        }

        int i = cur_waypoint;
        // add rest way points
        while (i < agent.plan.size()) {
            PoseStamped_ pose = new PoseStamped_();
            pose.setHeader(agent.plan.get(i).getHeader().copy());
            pose.setPose(agent.plan.get(i).getPose().copy());
            pose.getHeader().setStamp(agent.getBaseOdom().getHeader().getStamp());
            agent.transformed_plan.add(pose);
            ++i;
        }
        return true;
    }

    private static void stopWithAccLimits(Agent agent) {
        double vx = Methods_Planners.sign(agent.getBaseOdom().getTwist().getLinear().getX()) *
                Math.max(0.0, (Math.abs(agent.getBaseOdom().getTwist().getLinear().getX()) -
                        agent.acc_lim_x_ * agent.controlPeriod));
        double vy = Methods_Planners.sign(agent.getBaseOdom().getTwist().getLinear().getY()) *
                Math.max(0.0, (Math.abs(agent.getBaseOdom().getTwist().getLinear().getY()) -
                        agent.acc_lim_y_ * agent.controlPeriod));
        double vth = Methods_Planners.sign(agent.getBaseOdom().getTwist().getAngular().getZ()) *
                Math.max(0.0, (Math.abs(agent.getBaseOdom().getTwist().getAngular().getZ()) -
                        agent.acc_lim_th_ / agent.controlPeriod));

        agent.cmd_vel.getLinear().setX(vx);
        agent.cmd_vel.getLinear().setY(vy);
        agent.cmd_vel.getAngular().setZ(vth);
    }

    private static void rotateToGoal(Agent agent, double goal_th) {
        if (agent.ignore_goal_yaw) {
            agent.cmd_vel.getAngular().setZ(0);
        }
        double yaw = Methods_Planners.getYaw(agent.getBaseOdom().getPose().getOrientation());
        double vel_yaw = agent.getBaseOdom().getTwist().getAngular().getZ();
        agent.cmd_vel.getLinear().setX(0);
        agent.cmd_vel.getLinear().setY(0);

        double ang_diff = Methods_Planners.shortest_angular_distance(yaw, goal_th);

        double v_th_samp = ang_diff > 0.0 ?
                Math.min(agent.max_vel_th_, Math.max(agent.min_vel_th_inplace_, ang_diff)) :
                Math.max(-1.0 * agent.max_vel_th_, Math.min(-1.0 * agent.min_vel_th_inplace_, ang_diff));

        //take the acceleration limits of the robot into account
        double max_acc_vel = Math.abs(vel_yaw) + agent.acc_lim_th_ * agent.controlPeriod;
        double min_acc_vel = Math.abs(vel_yaw) - agent.acc_lim_th_ * agent.controlPeriod;

        v_th_samp = Methods_Planners.sign(v_th_samp) * Math.min(Math.max(Math.abs(v_th_samp), min_acc_vel), max_acc_vel);

        //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
        double max_speed_to_stop = Math.sqrt(2 * agent.acc_lim_th_ * Math.abs(ang_diff));//how to get this???

        v_th_samp = Methods_Planners.sign(v_th_samp) * Math.min(max_speed_to_stop, Math.abs(v_th_samp));

        if (Math.abs(v_th_samp) <= 0.0 * agent.min_vel_th_inplace_)//???????????
            v_th_samp = 0.0;
        else if (Math.abs(v_th_samp) < agent.min_vel_th_inplace_)
            v_th_samp = Methods_Planners.sign(v_th_samp) * Math.max(agent.min_vel_th_inplace_, Math.abs(v_th_samp));

        agent.cmd_vel.getAngular().setZ(v_th_samp);

    }

    private static void findBestWaypoint(Pose_ target_pose, Agent agent) {
        agent.current_waypoint = 0;
        double min_dist = Double.MAX_VALUE;
        for (int i = agent.current_waypoint; i < agent.transformed_plan.size(); i++) {
            double dist = Methods_Planners.getGoalPositionDistance(agent.getBaseOdom().getPose(),
                    agent.transformed_plan.get(i).getPose().getPosition().getX(),
                    agent.transformed_plan.get(i).getPose().getPosition().getY());
            if (dist < agent.footprint_radius_ || dist < min_dist) {
                min_dist = dist;
                target_pose.setPosition(agent.transformed_plan.get(i).getPose().getPosition());
                agent.current_waypoint = i;
            }
        }

        if (agent.current_waypoint == agent.transformed_plan.size() - 1) //I am at the end of the plan
            return;

        double dif_x = agent.transformed_plan.get(agent.current_waypoint + 1).
                getPose().getPosition().getX() - target_pose.getPosition().getX();
        double dif_y = agent.transformed_plan.get(agent.current_waypoint + 1).
                getPose().getPosition().getY() - target_pose.getPosition().getY();

        double plan_dir = Math.atan2(dif_y, dif_x);
        double dif_ang;

        for (int i = agent.current_waypoint + 1; i < agent.transformed_plan.size(); i++) {
            dif_x = agent.transformed_plan.get(i).getPose().getPosition().getX() - target_pose.getPosition().getX();
            dif_y = agent.transformed_plan.get(i).getPose().getPosition().getY() - target_pose.getPosition().getY();

            dif_ang = Math.atan2(dif_y, dif_x);

            if (Math.abs(Methods_Planners.normalize_angle(plan_dir - dif_ang)) > 1.0 * agent.yaw_goal_tolerance) {
                target_pose.setPosition(agent.transformed_plan.get(i - 1).getPose().getPosition().copy());
                agent.current_waypoint = i - 1;
                return;
            }
        }
        target_pose.setPosition(
                agent.transformed_plan.get(agent.transformed_plan.size() - 1).getPose().getPosition().copy()
        );
        agent.current_waypoint = agent.transformed_plan.size() - 1;
    }

    private static void checkVelocityCommand(Agent agent) {
        Methods_Planners.getValidCommand(agent);
        if (agent.cmd_vel.getLinear().getX() == 0.0 &&
                agent.cmd_vel.getAngular().getZ() == 0.0 &&
                agent.cmd_vel.getLinear().getY() == 0.0) {
            logger.info(agent.id + ": Did not find a valid velocity, trying next waypoint!");

            if (agent.current_waypoint < agent.transformed_plan.size() - 1) {
                agent.current_waypoint++;
                agent.skip_next = true;
            } else {
                agent.transformed_plan.clear();
                logger.warn(agent.id + ": Can not reach goal!");
            }
        } else {
            agent.skip_next = false;
        }

    }
}
