package cgl.iotrobots.collavoid.topologyStreaming;

import backtype.storm.Config;
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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class LocalPlannerBolt extends BaseRichBolt {
    private Logger logger = LoggerFactory.getLogger(LocalPlannerBolt.class);
    private OutputCollector outputCollector;
    private boolean reset = true;
    private Map<String, LocalPlannerContext> contexts = new HashMap<String, LocalPlannerContext>();
    private LocalPlannerContext currentContext;
    private String id;

    private class LocalPlannerContext {
        public String sensorID = null;
        public long time;
        public int current_waypoint = 0;
        public boolean skip_next = false;
        public boolean xy_tolerance_latch_ = false;
        public boolean rotating_to_goal_;
        public List<PoseStamped_> transformed_plan = new ArrayList<PoseStamped_>();
        public List<PoseStamped_> global_plan = null;
        public Odometry_ base_odom = null;
        public boolean locked = false;
        public boolean reachedGoal = true;
        public Vector2 pref_vel;
        public Twist_ cmd_vel;
        public Twist_ cmd_vel_cal;

        public LocalPlannerContext(String sensorID) {
            this.sensorID = sensorID;

        }
    }

    // every time emit a new tuple

    @Override
    public void execute(Tuple input) {
        id = input.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
        if (!contexts.containsKey(id)) {
            contexts.put(id, new LocalPlannerContext(id));
        }
        currentContext = contexts.get(id);


        // when ever reset the global plan stop the robot
        if (input.getSourceComponent().equals(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT)) {
            currentContext.global_plan = (List<PoseStamped_>) input.getValueByField(Constant_storm.FIELDS.PLAN_FIELD);
//            logger.warn("----------------------------"+sensorID+"received new plan");
            if (!transformGlobalPlan(currentContext.global_plan, currentContext.transformed_plan)) {
                logger.warn("{}: Could not transform the global plan to the frame of the controller",
                        currentContext.sensorID);
                currentContext.reachedGoal = true;
            } else {// reset global plan
                resetPlanner(currentContext);
                currentContext.reachedGoal = false;
            }
            outputCollector.emit(Constant_storm.Streams.VELOCITY_COMMAND_STREAM,
                    new Values(input.getValue(0), input.getValue(1), new Twist_()));
            // reset neighbor information, in practical situations this may need further consideration
            outputCollector.emit(Constant_storm.Streams.RESET_STREAM,
                    new Values(input.getValue(0), input.getValue(1), reset));
        } else if (!currentContext.reachedGoal) {
            if (input.getSourceStreamId().equals(Constant_storm.Streams.CONTROLLER_TIMER_STREAM)) {
                if (currentContext.locked) {
                    outputCollector.ack(input);
                    return;
                }
                currentContext.cmd_vel = null;
                currentContext.pref_vel = null;
                if (!computeVelocity()) {
                    logger.warn("{}: Invalid preferred velocity calculation!!", currentContext.sensorID);
                } else {
                    if (currentContext.pref_vel != null) {
                        outputCollector.emit(Constant_storm.Streams.PREFERRED_VELOCITY_STREAM, new Values(
                                input.getValueByField(Constant_storm.FIELDS.TIME_FIELD),
                                currentContext.sensorID,
                                currentContext.base_odom.copy(),
                                currentContext.pref_vel.copy()
                        ));
                        currentContext.locked = true;
                    } else {
                        outputCollector.emit(Constant_storm.Streams.VELOCITY_COMMAND_STREAM, new Values(
                                input.getValueByField(Constant_storm.FIELDS.TIME_FIELD),
                                currentContext.sensorID,
                                currentContext.cmd_vel.copy()
                        ));
                    }
                }
            } else {
                if (input.getSourceComponent().equals(Constant_storm.Components.ODOMETRY_TRANSFORM_COMPONENT)) {
                    currentContext.base_odom = (Odometry_) input.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
                } else if (currentContext.locked &&
                        input.getSourceComponent().equals(Constant_storm.Components.VELOCITY_COMPUTE_COMPONENT)) {
                    currentContext.cmd_vel_cal = (Twist_) input.getValueByField(Constant_storm.FIELDS.VELOCITY_COMMAND_FIELD);
                    currentContext.locked = false;
                    if (!checkVelocityCommand(currentContext.cmd_vel_cal)) {
                        logger.info("{}: Did not get available velocity, Try again!!", currentContext.sensorID);
                    } else {
                        outputCollector.emit(Constant_storm.Streams.VELOCITY_COMMAND_STREAM,
                                new Values(input.getLongByField(Constant_storm.FIELDS.TIME_FIELD),
                                        currentContext.sensorID,
                                        currentContext.cmd_vel_cal.copy()));
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
                        Constant_storm.FIELDS.ODOMETRY_FIELD,
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
        declarer.declareStream(Constant_storm.Streams.RESET_STREAM,
                new Fields(Constant_storm.FIELDS.TIME_FIELD,
                        Constant_storm.FIELDS.SENSOR_ID_FIELD,
                        Constant_storm.FIELDS.RESET_FIELD));
    }


    private boolean computeVelocity() {
        if (currentContext.base_odom == null) {
            logger.warn("{}: No odometry received yet!!", currentContext.sensorID);
            return false;
        }
        if (currentContext.global_plan == null) {
            logger.warn("{}: No global plan received yet!!", currentContext.sensorID);
            return false;
        }

        //get position and velocity,pose in global frame, velocity is in base frame
        PoseStamped_ global_pose = new PoseStamped_();
        global_pose.getHeader().setFrameId(Parameters.GLOBAL_FRAME);
        global_pose.getHeader().setStamp(System.currentTimeMillis());
        Twist_ base_vel;

        //velocity is in base frame
        //pose is in odometry or map frame
        global_pose.setPose(currentContext.base_odom.getPose().copy());
        base_vel = currentContext.base_odom.getTwist().copy();

        Pose_ goal_point;
        goal_point = currentContext.global_plan.get(currentContext.global_plan.size() - 1).getPose();
        double goal_x = goal_point.getPosition().getX();
        double goal_y = goal_point.getPosition().getY();
        double goal_th = Methods_Planners.getYaw(goal_point.getOrientation());

        //check to see if we've reached the goal position
        if (currentContext.xy_tolerance_latch_ ||
                (Methods_Planners.getGoalPositionDistance(
                        global_pose.getPose(),
                        goal_x,
                        goal_y) <= Parameters.XY_GOAL_TOLERANCE)) {

            //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
            //just rotate in place
            if (Parameters.LATCH_XY_GOAL_TOLERANCE)
                currentContext.xy_tolerance_latch_ = true;

            //check to see if the goal orientation has been reached
            double angle = Methods_Planners.getGoalOrientationAngleDifference(global_pose.getPose(), goal_th);

            currentContext.cmd_vel = new Twist_();
            //check to see if the goal orientation has been reached
            if (Math.abs(angle) <= Parameters.YAW_GOAL_TOLERANCE) {
                //set the velocity command to zero

                resetPlanner(currentContext);
                logger.info("{} reached the goal x: {}, y {}, theta {}.", currentContext.sensorID, goal_x, goal_y, goal_th);
                currentContext.reachedGoal = true;
            } else {
                //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
                if (!currentContext.rotating_to_goal_ && !Methods_Planners.stopped(
                        currentContext.base_odom,
                        Parameters.ROT_STOPPED_VELOCITY,
                        Parameters.TRANS_STOPPED_VELOCITY)) {
                    //ROS_DEBUG("Not stopped yet. base_odom: x=%6.4f,y=%6.4f,z=%6.4f", base_odom.twist.twist.linear.x,base_odom.twist.twist.linear.y,base_odom.twist.twist.angular.z);
                    stopWithAccLimits(base_vel, currentContext.cmd_vel);
                }
                //if we're stopped... then we want to rotate to goal
                else {
                    //set this so that we know its OK to be moving
                    currentContext.rotating_to_goal_ = true;
                    rotateToGoal(global_pose.getPose(), base_vel, goal_th, currentContext.cmd_vel);
                }
            }

            //publish an empty plan because we've reached our goal position
            currentContext.transformed_plan.clear();
            return true;
        }

        Pose_ target_pose = new Pose_();

        if (!currentContext.skip_next) {
            if (!transformGlobalPlan(currentContext.global_plan, currentContext.transformed_plan)) {
                logger.warn(currentContext.sensorID
                        + ": Could not transform the global plan to the frame of the controller");
                return false;
            }
            findBestWaypoint(target_pose, global_pose.getPose());
        } else {
            target_pose = currentContext.transformed_plan.get(currentContext.current_waypoint).getPose();
        }

        Twist_ pref_vel_twist = new Twist_();

        pref_vel_twist.getLinear().setX(target_pose.getPosition().getX() - global_pose.getPose().getPosition().getX());
        pref_vel_twist.getLinear().setY(target_pose.getPosition().getY() - global_pose.getPose().getPosition().getY());

        currentContext.pref_vel = new Vector2(pref_vel_twist.getLinear().getX(), pref_vel_twist.getLinear().getY());

        if (Vector2.abs(currentContext.pref_vel) > Parameters.MAX_VEL_X) {
            currentContext.pref_vel = Vector2.scale(Vector2.normalize(currentContext.pref_vel), Parameters.MAX_VEL_X);
        } else if (Vector2.abs(currentContext.pref_vel) < Parameters.MAX_VEL_X) {
            currentContext.pref_vel = Vector2.scale(
                    Vector2.normalize(currentContext.pref_vel),
                    Parameters.MAX_VEL_X * 1.2);
        }
        return true;
    }

    private void resetPlanner(LocalPlannerContext context) {
        if (context.cmd_vel != null) {
            context.cmd_vel.setLinear(new Vector3d_(0, 0, 0));
            context.cmd_vel.setAngular(new Vector3d_(0, 0, 0));
        }
        context.base_odom = null;
        context.skip_next = false;
        context.current_waypoint = 0;
        context.xy_tolerance_latch_ = false;
        context.rotating_to_goal_ = false;
        context.locked = false;
    }


    private boolean checkVelocityCommand(Twist_ cmd_vel) {
        if (Math.abs(cmd_vel.getAngular().getZ()) < Parameters.MIN_VEL_TH)
            cmd_vel.getAngular().setZ(0);

        if (Math.abs(cmd_vel.getLinear().getX()) < Parameters.MIN_VEL_X)
            cmd_vel.getLinear().setX(0);

        if (Math.abs(cmd_vel.getLinear().getY()) < Parameters.MIN_VEL_Y)
            cmd_vel.getLinear().setY(0);


        if (cmd_vel.getLinear().getX() == 0.0 && cmd_vel.getAngular().getZ() == 0.0 && cmd_vel.getLinear().getY() == 0.0) {
            logger.info(currentContext.sensorID + ": Did not find a good vel, trying next waypoint!");

            if (currentContext.current_waypoint < currentContext.transformed_plan.size() - 1) {
                currentContext.current_waypoint++;
                currentContext.skip_next = true;
            } else {
                currentContext.transformed_plan.clear();
                return false;
            }
        } else {
            currentContext.skip_next = false;
        }
        return true;
    }

    private boolean transformGlobalPlan(final List<PoseStamped_> global_plan, List<PoseStamped_> transformed_plan) {
        transformed_plan.clear();
        if (!(global_plan.size() > 0)) {
            logger.error("Recieved plan with zero length");
            return false;
        }
        //currently global plan is in global frame do not need transform, robot pose is also in global frame
        long t;
        Pose_ robot_pose;
        int cur_waypoint = 0;

        if (currentContext.base_odom != null) {
            robot_pose = currentContext.base_odom.getPose().copy();
            t = currentContext.base_odom.getHeader().getStamp();
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
            logger.warn("Odometry not received, consider initialization plan!!");
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
        cmd_vel.getLinear().setX(vx);
        cmd_vel.getLinear().setY(vy);
        cmd_vel.getAngular().setZ(vth);
    }

    private void rotateToGoal(final Pose_ global_pose, final Twist_ robot_vel, double goal_th, Twist_ cmd_vel) {
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

//        logger.info("Moving to desired goal orientation, th cmd: %1$2f" + v_th_samp);
        cmd_vel.getAngular().setZ(v_th_samp);

    }

    void findBestWaypoint(Pose_ target_pose, final Pose_ global_pose) {
        currentContext.current_waypoint = 0;
        double min_dist = Double.MAX_VALUE;
        for (int i = currentContext.current_waypoint; i < currentContext.transformed_plan.size(); i++) {
            double dist = Methods_Planners.getGoalPositionDistance(global_pose,
                    currentContext.transformed_plan.get(i).getPose().getPosition().getX(),
                    currentContext.transformed_plan.get(i).getPose().getPosition().getY());
            if (dist < Parameters.FOOTPRINT_RADIUS || dist < min_dist) {
                min_dist = dist;
                target_pose.setPosition(currentContext.transformed_plan.get(i).getPose().getPosition());
                currentContext.current_waypoint = i;
            }
        }
        //ROS_DEBUG("waypoint = %d, of %d", current_waypoint, transformed_plan.size());

        if (currentContext.current_waypoint == currentContext.transformed_plan.size() - 1) //I am at the end of the plan
            return;

        double dif_x = currentContext.transformed_plan.get(currentContext.current_waypoint + 1).
                getPose().getPosition().getX() - target_pose.getPosition().getX();
        double dif_y = currentContext.transformed_plan.get(currentContext.current_waypoint + 1).
                getPose().getPosition().getY() - target_pose.getPosition().getY();

        double plan_dir = Math.atan2(dif_y, dif_x);
        double dif_ang;

        for (int i = currentContext.current_waypoint + 1; i < currentContext.transformed_plan.size(); i++) {
            dif_x = currentContext.transformed_plan.get(i).getPose().getPosition().getX() - target_pose.getPosition().getX();
            dif_y = currentContext.transformed_plan.get(i).getPose().getPosition().getY() - target_pose.getPosition().getY();

            dif_ang = Math.atan2(dif_y, dif_x);

            if (Math.abs(Methods_Planners.normalize_angle(plan_dir - dif_ang)) > 1.0 * Parameters.YAW_GOAL_TOLERANCE) {
                target_pose.setPosition(currentContext.transformed_plan.get(i - 1).getPose().getPosition().copy());
                currentContext.current_waypoint = i - 1;
                return;
            }
        }
        target_pose.setPosition(
                currentContext.transformed_plan.get(currentContext.transformed_plan.size() - 1).getPose().getPosition().copy()
        );
        currentContext.current_waypoint = currentContext.transformed_plan.size() - 1;
    }


}
