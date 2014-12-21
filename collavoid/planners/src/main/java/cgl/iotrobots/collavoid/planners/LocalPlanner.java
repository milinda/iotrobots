package cgl.iotrobots.collavoid.planners;

import cgl.iotrobots.collavoid.commons.planners.Methods;
import cgl.iotrobots.collavoid.commons.planners.Parameters;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Pose_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Vector3d_;
import com.rabbitmq.client.Address;
//import costmap_2d.VoxelGrid;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

public class LocalPlanner {

    private Parameters parameters;

    String AgentName;

    boolean initialized_, skip_next_, setup_;

    double xy_goal_tolerance_, yaw_goal_tolerance_;

    double rot_stopped_velocity_, trans_stopped_velocity_;

    boolean latch_xy_goal_tolerance_, xy_tolerance_latch_, rotating_to_goal_, ignore_goal_yaw_;

    int current_waypoint_;

    public Agent me;

    private String global_frame_; ///< @brief The frame in which the controller will run

    private String base_frame_; ///< @brief Used as the base frame of the robot

    public List<Pose_> global_plan_, transformed_plan_;

    private static Logger logger;

    private Address[] Addresses;

    private String URL;

    /*---------------methods begin-----------------*/

    public LocalPlanner(String name,
                        Address[] addresses,
                        String url) {
        AgentName = name;
        Addresses = addresses;
        URL = url;
        initialized_ = false;
        current_waypoint_ = 0;
        skip_next_ = false;
        transformed_plan_ = new ArrayList<Pose_>();
        initialize();
    }

    //implement the interface of nav_core::BaseLocalPlanner Class in original ROS
    private void initialize() {
        if (!initialized_) {
            getParams();
            /*----------spawn agent node---------*/

            this.me = new Agent(AgentName, Addresses, URL);

            //wait for the agent to initialize
            while (!me.initialized_) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            initialized_ = true;
            logger.info("************************Local Planner" + AgentName + " is initialized!");
        } else {
            logger.info("************************Local Planner" + AgentName + " has already been initialized, you can't call it twice, doing nothing");
        }

    }

    // get parameters
    void getParams() {
        logger = Logger.getLogger(AgentName + "_Logger");

        //load parameters locally
        base_frame_ = AgentName + parameters.BASE_FRAME_SUFFIX;
        global_frame_ = parameters.GLOBAL_FRAME;
        rot_stopped_velocity_ = parameters.ROT_STOPPED_VELOCITY;
        trans_stopped_velocity_ = parameters.TRANS_STOPPED_VELOCITY;
        yaw_goal_tolerance_ = parameters.YAW_GOAL_TOLERANCE;
        xy_goal_tolerance_ = parameters.XY_GOAL_TOLERANCE;
        latch_xy_goal_tolerance_ = parameters.LATCH_XY_GOAL_TOLERANCE;
        ignore_goal_yaw_ = parameters.IGNORE_GOAL_YAW;

    }

    //implement interface setPlan
    public boolean setPlan(final List<Pose_> orig_global_plan) {
        if (!initialized_) {
            logger.severe("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        //reset the global plan
        global_plan_ = new ArrayList<Pose_>();
        global_plan_ = orig_global_plan;
        current_waypoint_ = 0;
        xy_tolerance_latch_ = false;
        //get the global plan, already in global frame no need to transform, need to fix
        if (!transformGlobalPlan(true, global_plan_, global_frame_, base_frame_, transformed_plan_)) {
            logger.warning("Could not transform the global plan to the frame of the controller");
            return false;
        }
        return true;
    }

    //implement interface isGoalReached
    public boolean isGoalReached() {
        if (!initialized_) {
            logger.severe("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //copy over the odometry information
        Odometry_ base_odom = new Odometry_();
        me.me_lock_.lock();
        try {
            base_odom = me.getBaseOdom();
        } finally {
            me.me_lock_.unlock();
        }

        Pose_ globalPose = base_odom.getPose();
        double distToGoal = Methods.getGoalPositionDistance(
                globalPose,
                global_plan_.get(global_plan_.size() - 1).getPosition().getX(),
                global_plan_.get(global_plan_.size() - 1).getPosition().getY()
        );

        double goal_th = Methods.getYaw(globalPose.getOrientation());
        double distToGoalangle = Methods.getGoalOrientationAngleDifference(global_plan_.get(global_plan_.size() - 1), goal_th);

        if (distToGoal <= parameters.XY_GOAL_TOLERANCE && distToGoalangle <= parameters.YAW_GOAL_TOLERANCE)
            if (base_odom.getTwist().getLinear().length() < parameters.EPSILON && base_odom.getTwist().getAngular().length() < parameters.EPSILON)
                return true;
        return false;
    }


    //implement interface for computeVelocityCommands
    public boolean computeVelocityCommands(Twist_ cmd_vel) {
        if (!initialized_) {
            logger.severe("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //get position and velocity,pose in global frame, velocity is in base frame
        Pose_ global_pose = new Pose_();
        global_pose.getHeader().setFrameId(global_frame_);
        global_pose.getHeader().setStamp(System.currentTimeMillis());
        Twist_ base_vel = new Twist_();

        //velocity is in base frame
        me.me_lock_.lock();
        try {
            base_vel.setLinear(me.getBaseOdom().getTwist().getLinear());
            base_vel.setAngular(me.getBaseOdom().getTwist().getAngular());
            //pose is in odometry or map frame
            global_pose.setPosition(me.getBaseOdom().getPose().getPosition());
            global_pose.setOrientation(me.getBaseOdom().getPose().getOrientation());
        } finally {
            me.me_lock_.unlock();
        }

        Pose_ goal_point;
        goal_point = global_plan_.get(global_plan_.size() - 1);
        double goal_x = goal_point.getPosition().getX();
        double goal_y = goal_point.getPosition().getY();
        double goal_th = Methods.getYaw(goal_point.getOrientation());

        //check to see if we've reached the goal position
        if (xy_tolerance_latch_ || (Methods.getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) {

            //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
            //just rotate in place
            if (latch_xy_goal_tolerance_)
                xy_tolerance_latch_ = true;

            //check to see if the goal orientation has been reached
            double angle = Methods.getGoalOrientationAngleDifference(global_pose, goal_th);

            //check to see if the goal orientation has been reached
            if (Math.abs(angle) <= yaw_goal_tolerance_) {
                //set the velocity command to zero
                cmd_vel.setLinear(new Vector3d_(0, 0, 0));
                cmd_vel.setAngular(new Vector3d_(0, 0, 0));
                rotating_to_goal_ = false;
                xy_tolerance_latch_ = false;
            } else {
                //copy over the odometry information
                Odometry_ base_odom = new Odometry_();
                me.me_lock_.lock();
                try {
                    base_odom = me.getBaseOdom();
                } finally {
                    me.me_lock_.unlock();
                }

                //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
                if (!rotating_to_goal_ && !Methods.stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)) {
                    //ROS_DEBUG("Not stopped yet. base_odom: x=%6.4f,y=%6.4f,z=%6.4f", base_odom.twist.twist.linear.x,base_odom.twist.twist.linear.y,base_odom.twist.twist.angular.z);
                    stopWithAccLimits(base_vel, cmd_vel);

                }
                //if we're stopped... then we want to rotate to goal
                else {
                    //set this so that we know its OK to be moving
                    rotating_to_goal_ = true;
                    rotateToGoal(global_pose, base_vel, goal_th, cmd_vel);
                }
            }

            //publish an empty plan because we've reached our goal position
            transformed_plan_.clear();
            return true;
        }

        Pose_ target_pose = new Pose_();

        if (!skip_next_) {
            if (!transformGlobalPlan(false, global_plan_, global_frame_, base_frame_, transformed_plan_)) {
                logger.warning("Could not transform the global plan to the frame of the controller");
                return false;
            }

            findBestWaypoint(target_pose, global_pose);
        }

        Twist_ pref_vel_twist = new Twist_();

        pref_vel_twist.getLinear().setX(target_pose.getPosition().getX() - global_pose.getPosition().getX());
        pref_vel_twist.getLinear().setY(target_pose.getPosition().getY() - global_pose.getPosition().getY());

        Vector2 pref_vel_vect = new Vector2(pref_vel_twist.getLinear().getX(), pref_vel_twist.getLinear().getY());

        if (Vector2.abs(pref_vel_vect) > me.max_vel_x_) {
            pref_vel_vect = Vector2.mul(Vector2.normalize(pref_vel_vect), me.max_vel_x_);
        } else if (Vector2.abs(pref_vel_vect) < me.min_vel_x_) {
            pref_vel_vect = Vector2.mul(Vector2.normalize(pref_vel_vect), me.min_vel_x_ * 1.2);
        }

        me.computeNewVelocity(pref_vel_vect, cmd_vel);


        if (Math.abs(cmd_vel.getAngular().getZ()) < me.min_vel_th_)
            cmd_vel.getAngular().setZ(0);

        if (Math.abs(cmd_vel.getLinear().getX()) < me.min_vel_x_)
            cmd_vel.getLinear().setX(0);

        if (Math.abs(cmd_vel.getLinear().getY()) < me.min_vel_y_)
            cmd_vel.getLinear().setY(0);


        if (cmd_vel.getLinear().getX() == 0.0 && cmd_vel.getAngular().getZ() == 0.0 && cmd_vel.getLinear().getY() == 0.0) {
            logger.fine("Did not find a good vel, calculated best holonomic velocity was:"
                    + me.velocity.getX() + ", "
                    + me.velocity.getY() + ", cur wp "
                    + current_waypoint_ + " of "
                    + transformed_plan_.size()
                    + " trying next waypoint");

            if (current_waypoint_ < transformed_plan_.size() - 1) {
                current_waypoint_++;
                skip_next_ = true;
            } else {
                transformed_plan_.clear();
                return false;
            }
        } else {
            skip_next_ = false;
        }

        // for visualization
        List<Pose_> local_plan = new ArrayList<Pose_>();
        Pose_ pos = new Pose_();

        pos.setPose(global_pose);//global pose is in base frame???
        local_plan.add(pos);
        local_plan.add(transformed_plan_.get(current_waypoint_));

        return true;
    }


    void stopWithAccLimits(final Twist_ robot_vel, Twist_ cmd_vel) {
        double vx = Methods.sign(robot_vel.getLinear().getX()) * Math.max(0.0, (Math.abs(robot_vel.getLinear().getX()) - me.acc_lim_x_ * me.simPeriod));
        double vy = Methods.sign(robot_vel.getLinear().getY()) * Math.max(0.0, (Math.abs(robot_vel.getLinear().getY()) - me.acc_lim_y_ * me.simPeriod));
        double vth = Methods.sign(robot_vel.getAngular().getZ()) * Math.max(0.0, (Math.abs(robot_vel.getAngular().getZ()) - me.acc_lim_th_ * me.simPeriod));

        //ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
        cmd_vel.getLinear().setX(vx);
        cmd_vel.getLinear().setY(vy);
        cmd_vel.getAngular().setZ(vth);
    }


    void rotateToGoal(final Pose_ global_pose, final Twist_ robot_vel, double goal_th, Twist_ cmd_vel) {
        if (ignore_goal_yaw_) {
            cmd_vel.getAngular().setZ(0);
        }
        double yaw = Methods.getYaw(global_pose.getOrientation());
        double vel_yaw = robot_vel.getAngular().getZ();
        cmd_vel.getLinear().setX(0);
        cmd_vel.getLinear().setY(0);

        double ang_diff = Methods.shortest_angular_distance(yaw, goal_th);

        double v_th_samp = ang_diff > 0.0 ?
                Math.min(me.max_vel_th_, Math.max(me.min_vel_th_inplace_, ang_diff)) :
                Math.max(-1.0 * me.max_vel_th_, Math.min(-1.0 * me.min_vel_th_inplace_, ang_diff));

        //take the acceleration limits of the robot into account
        double max_acc_vel = Math.abs(vel_yaw) + me.acc_lim_th_ * me.simPeriod;
        double min_acc_vel = Math.abs(vel_yaw) - me.acc_lim_th_ * me.simPeriod;

        v_th_samp = Methods.sign(v_th_samp) * Math.min(Math.max(Math.abs(v_th_samp), min_acc_vel), max_acc_vel);

        //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
        double max_speed_to_stop = Math.sqrt(2 * me.acc_lim_th_ * Math.abs(ang_diff));//how to get this???

        v_th_samp = Methods.sign(v_th_samp) * Math.min(max_speed_to_stop, Math.abs(v_th_samp));
        if (Math.abs(v_th_samp) <= 0.0 * me.min_vel_th_inplace_)//???????????
            v_th_samp = 0.0;
        else if (Math.abs(v_th_samp) < me.min_vel_th_inplace_)
            v_th_samp = Methods.sign(v_th_samp) * Math.max(me.min_vel_th_inplace_, Math.abs(v_th_samp));

        logger.fine("Moving to desired goal orientation, th cmd: %1$2f" + v_th_samp);
        cmd_vel.getAngular().setZ(v_th_samp);

    }

    // transform global plan
    boolean transformGlobalPlan(boolean initialPlan, final List<Pose_> global_plan, final String global_frame, final String base_frame, List<Pose_> transformed_plan) {

        Pose_ plan_pose = new Pose_();

        plan_pose.setPosition(global_plan.get(0).getPosition());
        plan_pose.setHeader(global_plan.get(0).getHeader());

        transformed_plan.clear();

        if (!(global_plan.size() > 0)) {
            logger.severe("Recieved plan with zero length");
            return false;
        }

        //currently global plan is in global frame do not need transform
        long t;
        Pose_ robot_pose = new Pose_();
        int cur_waypoint = 0;
        if (!initialPlan) {
            me.me_lock_.lock();
            try {
                robot_pose = me.getBaseOdom().getPose();
                if (me.getLastSeen() == 0L)
                    t = System.currentTimeMillis();
                else
                    t = me.getLastSeen();
            } finally {
                me.me_lock_.unlock();
            }

            //we'll keep points on the plan that are within the window that we're looking at

            double sq_dist = Double.MAX_VALUE;

            double dist;

            for (int i = 0; i < global_plan_.size(); i++) {
                dist = Methods.getGoalPositionDistance(
                        robot_pose,
                        global_plan_.get(i).getPosition().getX(),
                        global_plan_.get(i).getPosition().getY());

                if (dist < Math.sqrt(sq_dist)) {
                    sq_dist = dist * dist;
                    cur_waypoint = i;
                }
            }

        } else {
            t = System.currentTimeMillis();
        }

        int i = cur_waypoint;

        //now we'll transform until points are outside of our distance threshold
        while (i < global_plan.size()) {
            Pose_ pose = new Pose_();
            pose.setHeader(global_plan.get(i).getHeader());
            pose.getHeader().setStamp(t);
            pose.setPosition(global_plan.get(i).getPosition());
            transformed_plan.add(pose);
            ++i;
        }
        return true;
    }


    void findBestWaypoint(Pose_ target_pose, final Pose_ global_pose) {
        current_waypoint_ = 0;
        double min_dist = Double.MAX_VALUE;
        for (int i = current_waypoint_; i < transformed_plan_.size(); i++) {
            double dist = Methods.getGoalPositionDistance(global_pose, transformed_plan_.get(i).getPosition().getX(),
                    transformed_plan_.get(i).getPosition().getY());
            if (dist < me.getRadius() || dist < min_dist) {
                min_dist = dist;
                target_pose.setPose(transformed_plan_.get(i));
                current_waypoint_ = i;
            }
        }
        //ROS_DEBUG("waypoint = %d, of %d", current_waypoint_, transformed_plan_.size());

        if (current_waypoint_ == transformed_plan_.size() - 1) //I am at the end of the plan
            return;

        double dif_x = transformed_plan_.get(current_waypoint_ + 1).getPosition().getX() - target_pose.getPosition().getX();
        double dif_y = transformed_plan_.get(current_waypoint_ + 1).getPosition().getY() - target_pose.getPosition().getY();

        double plan_dir = Math.atan2(dif_y, dif_x);
        double dif_ang;

        for (int i = current_waypoint_ + 1; i < transformed_plan_.size(); i++) {
            dif_x = transformed_plan_.get(i).getPosition().getX() - target_pose.getPosition().getX();
            dif_y = transformed_plan_.get(i).getPosition().getY() - target_pose.getPosition().getY();

            dif_ang = Math.atan2(dif_y, dif_x);

            if (Math.abs(plan_dir - dif_ang) > 1.0 * yaw_goal_tolerance_) {
                target_pose.setPose(transformed_plan_.get(i - 1));
                current_waypoint_ = i - 1;
                return;
            }
        }
        target_pose.setPose(transformed_plan_.get(transformed_plan_.size() - 1));
        current_waypoint_ = transformed_plan_.size() - 1;
    }

}

