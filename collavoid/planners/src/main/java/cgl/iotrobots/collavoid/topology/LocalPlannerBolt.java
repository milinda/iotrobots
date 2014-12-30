package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.planners.Methods_Planners;
import cgl.iotrobots.collavoid.commons.planners.Parameters;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;

public class LocalPlannerBolt extends BaseRichBolt {
    private Logger logger = Logger.getLogger("LocalPlannerBolt");
    private OutputCollector outputCollector;
    private Object sensorID = null;
    private Object time;
    private int current_waypoint = 0;
    private boolean skip_next = false;
    private boolean xy_tolerance_latch_ = false;
    private boolean rotating_to_goal_;
    private List<PoseStamped_> transformed_plan = new ArrayList<PoseStamped_>();
    private List<PoseStamped_> global_plan = null;
    private Odometry_ base_odom = null;
    private boolean locked = false;
    private boolean reachedGoal = true;

    @Override
    public void execute(Tuple input) {
        // when ever reset the global plan stop the robot
        if (input.getSourceComponent().equals(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT)) {
            global_plan = (List<PoseStamped_>) input.getValueByField(Constant_storm.FIELDS.PLAN_FIELD);
            if (!transformGlobalPlan(global_plan, transformed_plan)) {
                logger.warning("Could not transform the global plan to the frame of the controller");
            } else {// reset global plan
                skip_next = false;
                current_waypoint = 0;
                xy_tolerance_latch_ = false;
            }
            Twist_ cmd_vel_stop = new Twist_();
            resetPlanner(cmd_vel_stop);
            reachedGoal = false;
            outputCollector.emit(Constant_storm.Streams.VELOCITY_COMMAND_STREAM,
                    new Values(input.getValue(0), input.getValue(1), cmd_vel_stop));
        }

        if (!reachedGoal) {
            if (input.getSourceStreamId().equals(Constant_storm.Streams.CONTROLLER_TIMER_STREAM)) {
                if (locked) {
                    outputCollector.ack(input);
                    return;
                }
                Twist_ cmd_vel = null;
                Vector2 pre_vel = null;
                if (!computeVelocity(pre_vel, cmd_vel)) {
                    logger.warning("In valid preferred velocity calculation!!");
                } else {
                    if (sensorID == null)
                        return;
                    if (pre_vel != null) {
                        outputCollector.emit(Constant_storm.Streams.PREFERRED_VELOCITY_STREAM, new Values(
                                input.getValueByField(Constant_storm.FIELDS.TIME_FIELD),
                                sensorID,
                                pre_vel
                        ));
                        locked = true;
                    } else {
                        outputCollector.emit(Constant_storm.Streams.VELOCITY_COMMAND_STREAM, new Values(
                                input.getValueByField(Constant_storm.FIELDS.TIME_FIELD),
                                sensorID,
                                cmd_vel
                        ));
                    }
                }
            } else {
                sensorID = input.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
                time = input.getValueByField(Constant_storm.FIELDS.TIME_FIELD);
                if (input.getSourceComponent().equals(Constant_storm.Components.ODOMETRY_COMPONENT)) {
                    base_odom = (Odometry_) input.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
                } else if (locked && input.getSourceComponent().equals(Constant_storm.Components.AGENT_COMPONENT)) {
                    Twist_ cmd_vel = (Twist_) input.getValueByField(Constant_storm.FIELDS.VELOCITY_COMMAND_FIELD);
                    locked = false;
                    if (!checkVelocityCommand(cmd_vel)) {
                        logger.info("Already reached the goal!!");
                        reachedGoal = true;
                    } else {
                        List<Object> emit = new ArrayList<Object>();
                        emit.add(time);
                        emit.add(sensorID);
                        emit.add(cmd_vel);
                        outputCollector.emit(Constant_storm.Streams.VELOCITY_COMMAND_STREAM, emit);
                    }
                }
            }
        }

        outputCollector.ack(input);
    }

    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        outputCollector = collector;
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declareStream(Constant_storm.Streams.PREFERRED_VELOCITY_STREAM,
                new Fields(
                        Constant_storm.FIELDS.TIME_FIELD,
                        Constant_storm.FIELDS.SENSOR_ID_FIELD,
                        Constant_storm.FIELDS.PREFERRED_VELOCITY_FIELD
                )
        );
        declarer.declareStream(Constant_storm.Streams.VELOCITY_COMMAND_STREAM,
                new Fields(
                        Constant_storm.FIELDS.TIME_FIELD,
                        Constant_storm.FIELDS.SENSOR_ID_FIELD,
                        Constant_storm.FIELDS.VELOCITY_COMMAND_FIELD
                )
        );
    }


    private boolean computeVelocity(Vector2 pre_vel, Twist_ cmd_vel) {
        if (base_odom == null) {
            logger.severe("No odometry received yet!!");
            return false;
        }
        if (global_plan == null) {
            logger.severe("No global plan received yet!!");
            return false;
        }

        //get position and velocity,pose in global frame, velocity is in base frame
        PoseStamped_ global_pose = new PoseStamped_();
        global_pose.getHeader().setFrameId(Parameters.GLOBAL_FRAME);
        global_pose.getHeader().setStamp(System.currentTimeMillis());
        Twist_ base_vel;

        //velocity is in base frame
        //pose is in odometry or map frame
        global_pose.setPose(base_odom.getPose().copy());
        base_vel = base_odom.getTwist().copy();

        Pose_ goal_point;
        goal_point = global_plan.get(global_plan.size() - 1).getPose();
        double goal_x = goal_point.getPosition().getX();
        double goal_y = goal_point.getPosition().getY();
        double goal_th = Methods_Planners.getYaw(goal_point.getOrientation());

        //check to see if we've reached the goal position
        if (xy_tolerance_latch_ ||
                (Methods_Planners.getGoalPositionDistance(
                        global_pose.getPose(),
                        goal_x,
                        goal_y) <= Parameters.XY_GOAL_TOLERANCE)) {

            //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
            //just rotate in place
            if (Parameters.LATCH_XY_GOAL_TOLERANCE)
                xy_tolerance_latch_ = true;

            //check to see if the goal orientation has been reached
            double angle = Methods_Planners.getGoalOrientationAngleDifference(global_pose.getPose(), goal_th);

            //check to see if the goal orientation has been reached
            if (Math.abs(angle) <= Parameters.YAW_GOAL_TOLERANCE) {
                //set the velocity command to zero
                cmd_vel = new Twist_();
                resetPlanner(cmd_vel);
                reachedGoal = true;
            } else {
                //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
                if (!rotating_to_goal_ && !Methods_Planners.stopped(
                        base_odom,
                        Parameters.ROT_STOPPED_VELOCITY,
                        Parameters.TRANS_STOPPED_VELOCITY)) {
                    //ROS_DEBUG("Not stopped yet. base_odom: x=%6.4f,y=%6.4f,z=%6.4f", base_odom.twist.twist.linear.x,base_odom.twist.twist.linear.y,base_odom.twist.twist.angular.z);
                    stopWithAccLimits(base_vel, cmd_vel);

                }
                //if we're stopped... then we want to rotate to goal
                else {
                    //set this so that we know its OK to be moving
                    rotating_to_goal_ = true;
                    rotateToGoal(global_pose.getPose(), base_vel, goal_th, cmd_vel);
                }
            }

            //publish an empty plan because we've reached our goal position
            transformed_plan.clear();
            return true;
        }

        Pose_ target_pose = new Pose_();

        if (!skip_next) {
            if (!transformGlobalPlan(global_plan, transformed_plan)) {
                logger.warning("Could not transform the global plan to the frame of the controller");
                return false;
            }
            findBestWaypoint(target_pose, global_pose.getPose());
        } else {
            target_pose = transformed_plan.get(current_waypoint).getPose();
        }

        Twist_ pref_vel_twist = new Twist_();

        pref_vel_twist.getLinear().setX(target_pose.getPosition().getX() - global_pose.getPose().getPosition().getX());
        pref_vel_twist.getLinear().setY(target_pose.getPosition().getY() - global_pose.getPose().getPosition().getY());

        pre_vel = new Vector2(pref_vel_twist.getLinear().getX(), pref_vel_twist.getLinear().getY());

        if (Vector2.abs(pre_vel) > Parameters.MAX_VEL_X) {
            pre_vel = Vector2.scale(Vector2.normalize(pre_vel), Parameters.MAX_VEL_X);
        } else if (Vector2.abs(pre_vel) < Parameters.MAX_VEL_X) {
            pre_vel = Vector2.scale(Vector2.normalize(pre_vel), Parameters.MAX_VEL_X * 1.2);
        }
        return true;
    }

    private void resetPlanner(Twist_ cmd_vel) {
        cmd_vel.setLinear(new Vector3d_(0, 0, 0));
        cmd_vel.setAngular(new Vector3d_(0, 0, 0));
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
        locked = false;
    }


    private boolean checkVelocityCommand(Twist_ cmd_vel) {
        if (Math.abs(cmd_vel.getAngular().getZ()) < Parameters.MIN_VEL_TH)
            cmd_vel.getAngular().setZ(0);

        if (Math.abs(cmd_vel.getLinear().getX()) < Parameters.MIN_VEL_X)
            cmd_vel.getLinear().setX(0);

        if (Math.abs(cmd_vel.getLinear().getY()) < Parameters.MIN_VEL_Y)
            cmd_vel.getLinear().setY(0);


        if (cmd_vel.getLinear().getX() == 0.0 && cmd_vel.getAngular().getZ() == 0.0 && cmd_vel.getLinear().getY() == 0.0) {
            logger.fine("Did not find a good vel, trying next waypoint!");

            if (current_waypoint < transformed_plan.size() - 1) {
                current_waypoint++;
                skip_next = true;
            } else {
                transformed_plan.clear();
                return false;
            }
        } else {
            skip_next = false;
        }
        return true;
    }

    private boolean transformGlobalPlan(final List<PoseStamped_> global_plan, List<PoseStamped_> transformed_plan) {
        transformed_plan.clear();
        if (!(global_plan.size() > 0)) {
            logger.severe("Recieved plan with zero length");
            return false;
        }
        //currently global plan is in global frame do not need transform, robot pose is also in global frame
        long t;
        Pose_ robot_pose;
        int cur_waypoint = 0;

        if (base_odom != null) {
            robot_pose = base_odom.getPose().copy();
            t = base_odom.getHeader().getStamp();
            //we'll keep points on the plan that are within the window that we're looking at
            double sq_dist = Double.MAX_VALUE;
            double dist;
            for (int i = 0; i < global_plan.size(); i++) {
                dist = Methods_Planners.getGoalPositionDistance(
                        robot_pose,
                        global_plan.get(i).getPose().getPosition().getX(),
                        global_plan.get(i).getPose().getPosition().getY()
                );
                if (dist < Math.sqrt(sq_dist)) {
                    sq_dist = dist * dist;
                    cur_waypoint = i;
                }
            }
        } else {
            logger.warning("Odometry not received, consider initialization plan!!");
            t = System.currentTimeMillis();
        }

        int i = cur_waypoint;
        // add rest way points
        while (i < global_plan.size()) {
            PoseStamped_ pose = new PoseStamped_();
            pose.setHeader(global_plan.get(i).getHeader().copy());
            pose.setPose(global_plan.get(i).getPose().copy());
            pose.getHeader().setStamp(t);
            transformed_plan.add(pose);
            ++i;
        }
        return true;
    }

    private void stopWithAccLimits(final Twist_ robot_vel, Twist_ cmd_vel) {
        double vx = Methods_Planners.sign(robot_vel.getLinear().getX()) *
                Math.max(0.0, (Math.abs(robot_vel.getLinear().getX()) -
                        Parameters.ACC_LIM_X / Parameters.CONTROLLER_FREQUENCY));
        double vy = Methods_Planners.sign(robot_vel.getLinear().getY()) *
                Math.max(0.0, (Math.abs(robot_vel.getLinear().getY()) -
                        Parameters.ACC_LIM_Y / Parameters.CONTROLLER_FREQUENCY));
        double vth = Methods_Planners.sign(robot_vel.getAngular().getZ()) *
                Math.max(0.0, (Math.abs(robot_vel.getAngular().getZ()) -
                        Parameters.ACC_LIM_TH / Parameters.CONTROLLER_FREQUENCY));

        //ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
        cmd_vel = new Twist_();
        cmd_vel.getLinear().setX(vx);
        cmd_vel.getLinear().setY(vy);
        cmd_vel.getAngular().setZ(vth);
    }

    private void rotateToGoal(final Pose_ global_pose, final Twist_ robot_vel, double goal_th, Twist_ cmd_vel) {
        cmd_vel = new Twist_();
        if (Parameters.IGNORE_GOAL_YAW) {
            cmd_vel.getAngular().setZ(0);
        }
        double yaw = Methods_Planners.getYaw(global_pose.getOrientation());
        double vel_yaw = robot_vel.getAngular().getZ();
        cmd_vel.getLinear().setX(0);
        cmd_vel.getLinear().setY(0);

        double ang_diff = Methods_Planners.shortest_angular_distance(yaw, goal_th);

        double v_th_samp = ang_diff > 0.0 ?
                Math.min(Parameters.MAX_VEL_TH, Math.max(Parameters.MIN_VEL_TH_INPLACE, ang_diff)) :
                Math.max(-1.0 * Parameters.MAX_VEL_TH, Math.min(-1.0 * Parameters.MIN_VEL_TH_INPLACE, ang_diff));

        //take the acceleration limits of the robot into account
        double max_acc_vel = Math.abs(vel_yaw) + Parameters.ACC_LIM_TH / Parameters.CONTROLLER_FREQUENCY;
        double min_acc_vel = Math.abs(vel_yaw) - Parameters.ACC_LIM_TH / Parameters.CONTROLLER_FREQUENCY;

        v_th_samp = Methods_Planners.sign(v_th_samp) * Math.min(Math.max(Math.abs(v_th_samp), min_acc_vel), max_acc_vel);

        //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
        double max_speed_to_stop = Math.sqrt(2 * Parameters.ACC_LIM_TH * Math.abs(ang_diff));//how to get this???

        v_th_samp = Methods_Planners.sign(v_th_samp) * Math.min(max_speed_to_stop, Math.abs(v_th_samp));
        if (Math.abs(v_th_samp) <= 0.0 * Parameters.MIN_VEL_TH_INPLACE)//???????????
            v_th_samp = 0.0;
        else if (Math.abs(v_th_samp) < Parameters.MIN_VEL_TH_INPLACE)
            v_th_samp = Methods_Planners.sign(v_th_samp) * Math.max(Parameters.MIN_VEL_TH_INPLACE, Math.abs(v_th_samp));

        logger.fine("Moving to desired goal orientation, th cmd: %1$2f" + v_th_samp);
        cmd_vel.getAngular().setZ(v_th_samp);

    }

    void findBestWaypoint(Pose_ target_pose, final Pose_ global_pose) {
        current_waypoint = 0;
        double min_dist = Double.MAX_VALUE;
        for (int i = current_waypoint; i < transformed_plan.size(); i++) {
            double dist = Methods_Planners.getGoalPositionDistance(global_pose,
                    transformed_plan.get(i).getPose().getPosition().getX(),
                    transformed_plan.get(i).getPose().getPosition().getY());
            if (dist < Parameters.FOOTPRINT_RADIUS || dist < min_dist) {
                min_dist = dist;
                target_pose.setPosition(transformed_plan.get(i).getPose().getPosition());
                current_waypoint = i;
            }
        }
        //ROS_DEBUG("waypoint = %d, of %d", current_waypoint, transformed_plan.size());

        if (current_waypoint == transformed_plan.size() - 1) //I am at the end of the plan
            return;

        double dif_x = transformed_plan.get(current_waypoint + 1).getPose().getPosition().getX() - target_pose.getPosition().getX();
        double dif_y = transformed_plan.get(current_waypoint + 1).getPose().getPosition().getY() - target_pose.getPosition().getY();

        double plan_dir = Math.atan2(dif_y, dif_x);
        double dif_ang;

        for (int i = current_waypoint + 1; i < transformed_plan.size(); i++) {
            dif_x = transformed_plan.get(i).getPose().getPosition().getX() - target_pose.getPosition().getX();
            dif_y = transformed_plan.get(i).getPose().getPosition().getY() - target_pose.getPosition().getY();

            dif_ang = Math.atan2(dif_y, dif_x);

            if (Math.abs(Methods_Planners.normalize_angle(plan_dir - dif_ang)) > 1.0 * Parameters.YAW_GOAL_TOLERANCE) {
                target_pose.setPosition(transformed_plan.get(i - 1).getPose().getPosition().copy());
                current_waypoint = i - 1;
                return;
            }
        }
        target_pose.setPosition(transformed_plan.get(transformed_plan.size() - 1).getPose().getPosition().copy());
        current_waypoint = transformed_plan.size() - 1;
    }


}
