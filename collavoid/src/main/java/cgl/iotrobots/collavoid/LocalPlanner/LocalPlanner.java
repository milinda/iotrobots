package cgl.iotrobots.collavoid.LocalPlanner;

import cgl.iotrobots.collavoid.ROSAgent.ROSAgent;
import cgl.iotrobots.collavoid.utils.Angles;
import cgl.iotrobots.collavoid.utils.Parameters;
import cgl.iotrobots.collavoid.utils.Vector2;
import costmap_2d.VoxelGrid;
import geometry_msgs.*;
import nav_msgs.OccupancyGrid;
import nav_msgs.Odometry;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.*;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.pubsub.TransformListener;
import visualization_msgs.MarkerArray;

import java.lang.String;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

import static cgl.iotrobots.collavoid.utils.utils.sign;


public class LocalPlanner {
    //currently no dynamic reconfigurer

    //rosjava node params
    String id;
    protected ConnectedNode node;
    private ParameterTree params;

    //costmap Datatypes:
    VoxelGrid costmap_ros_;
    VoxelGrid costmap_; ///< @brief The costmap the controller will use

    TransformListener tf_;

    boolean initialized_, skip_next_, setup_;

    //ROSAgent stuff
//  double simPeriod;
//    double max_vel_x_, min_vel_x_;
//    double max_vel_y_, min_vel_y_;
//    double max_vel_th_, min_vel_th_, min_vel_th_inplace_;
//    double acc_lim_x_, acc_lim_y_, acc_lim_th_;
    double wheel_base_, radius_;
//    double max_vel_with_obstacles_;

    boolean holo_robot_;

    double trunc_time_, left_pref_;

    double xy_goal_tolerance_, yaw_goal_tolerance_;
    double rot_stopped_velocity_, trans_stopped_velocity_;

    double publish_me_period_, publish_positions_period_;
    double threshold_last_seen_;

    boolean latch_xy_goal_tolerance_, xy_tolerance_latch_, rotating_to_goal_, ignore_goal_yaw_, delete_observations_;

    //originally should be unsigned
    int current_waypoint_;
    //params ORCA
    double time_horizon_obst_;
    double eps_;

    public ROSAgent me;

    double time_to_holo_, min_error_holo_, max_error_holo_;

    private String global_frame_; ///< @brief The frame in which the controller will run
    private String base_frame_; ///< @brief Used as the base frame id of the robot
    public List<PoseStamped> global_plan_, transformed_plan_;
    Publisher g_plan_pub_, l_plan_pub_;
    Subscriber obstacles_sub_; //not used
    String hostname;

    private static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    private static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);
    private static Vector3 linear = messageFactory.newFromType(Vector3._TYPE);
    private static Vector3 angular = messageFactory.newFromType(Vector3._TYPE);//for temporary assign use
    private static Logger logger;


    /*---------------methods begin-----------------*/

    //must pass connectednode,tf will be initialized from the caller, see rosjava_tf TfViz
    public LocalPlanner(ConnectedNode connectedNode, TransformListener tf) {
        node = connectedNode;
        tf_ = tf;

        id = node.getName().toString().replace("/planner_", "");
        initialized_ = false;
        costmap_ros_ = null;
        current_waypoint_ = 0;
        skip_next_ = false;

        transformed_plan_ = new ArrayList<PoseStamped>();

        initialize();
    }

    //implement the interface of nav_core::BaseLocalPlanner Class in original ROS
    private void initialize() {
        if (!initialized_) {
            getParams(false);
            /*----------spawn agent node---------*/

            this.me = new ROSAgent(this.node, tf_);
            radius_ = me.footprint_radius_;

            /*------------------ Publishers----------------- */
            g_plan_pub_ = this.node.newPublisher(id + "/g_plan_pub_", MarkerArray._TYPE);
            //g_plan_pub_.setLatchMode(true);
            l_plan_pub_ = this.node.newPublisher(id + "/l_plan_pub_", MarkerArray._TYPE);
            //l_plan_pub_.setLatchMode(true);
            //ROS_ERROR("%s name of node", thisname.c_str());

            /*------------------Subscribers------------*/
            //String move_base_name = this.node.getName().toString();
            //Subscriber<OccupancyGrid> obstacles_sub_ = this.node.newSubscriber(move_base_name + "/local_costmap/obstacles", OccupancyGrid._TYPE);
            //it is not used
//            Subscriber<OccupancyGrid> obstacles_sub_ = this.node.newSubscriber("/move_base/local_costmap/costmap", OccupancyGrid._TYPE);
//            obstacles_sub_.addMessageListener(new MessageListener<OccupancyGrid>() {
//                @Override
//                public void onNewMessage(OccupancyGrid msg) {
//                    obstaclesCallback(msg);
//                }
//            });

            //setup_= false;??????????????
            //not implemented yet
            //dsrv_ = new dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>(private_nh);
            //dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>::CallbackType cb = boost::bind(&CollvoidLocalPlanner::reconfigureCB, this, _1, _2);
            //dsrv_->setCallback(cb);

            //wait for the agent to initialize
            while (!me.initialized_) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            initialized_ = true;
            node.getLog().info("************************Local Planner" + node.getName() + " is initialized!");
        } else {
            node.getLog().info("************************Local Planner" + node.getName() + " has already been initialized, you can't call it twice, doing nothing");
        }

    }

    // get parameters
    void getParams(boolean useROSParamService) {
        logger = Logger.getLogger(node.getName().toString());

        if (useROSParamService) {
            // use parameter services
            params = node.getParameterTree();
            base_frame_ = params.getString("/base_frame", id + "_base");
            global_frame_ = params.getString("/global_frame", "map");
            rot_stopped_velocity_ = params.getDouble("/rot_stopped_velocity", 0.01);
            trans_stopped_velocity_ = params.getDouble("/trans_stopped_velocity_", 0.01);
            yaw_goal_tolerance_ = params.getDouble("/yaw_goal_tolerance", 0.05);
            xy_goal_tolerance_ = params.getDouble("/xy_goal_tolerance", 0.10);
            latch_xy_goal_tolerance_ = params.getBoolean("/latch_xy_goal_tolerance", false);
            ignore_goal_yaw_ = params.getBoolean("/ignore_goal_yaw", false);
        } else {
            //load parameters locally
            base_frame_ = id + Parameters.BASE_FRAME_SUFFIX;
            global_frame_ = Parameters.GLOBAL_FRAME;
            rot_stopped_velocity_ = Parameters.ROT_STOPPED_VELOCITY;
            trans_stopped_velocity_ = Parameters.TRANS_STOPPED_VELOCITY;
            yaw_goal_tolerance_ = Parameters.YAW_GOAL_TOLERANCE;
            xy_goal_tolerance_ = Parameters.XY_GOAL_TOLERANCE;
            latch_xy_goal_tolerance_ = Parameters.LATCH_XY_GOAL_TOLERANCE;
            ignore_goal_yaw_ = Parameters.IGNORE_GOAL_YAW;
        }
    }

    /*Add obstacles from map, not used*/
    void obstaclesCallback(final OccupancyGrid msg) {
        Vector2 origin = new Vector2(msg.getInfo().getOrigin().getPosition().getX(), msg.getInfo().getOrigin().getPosition().getY());
        int mapWith = msg.getInfo().getWidth();
        double resolution = msg.getInfo().getResolution();
        int occupancyThreshold = 50;
        List<Vector2> points = new ArrayList<Vector2>();
        for (int i = 0; i < msg.getData().readableBytes(); i++) {
            if (msg.getData().getByte(i) > occupancyThreshold) {
                int row = i % mapWith;
                int col = i / mapWith;
                double x = row * resolution;
                double y = col * resolution;
                //TODO:need transform to base frame!!!!
                points.add(Vector2.plus(origin, new Vector2(x, y)));
            }
        }
        me.obstacle_lock_.lock();
        try {
            me.obstacle_points_.clear();
            for (Vector2 pt : points) {
                me.obstacle_points_.add(pt);
            }
        } finally {
            me.obstacle_lock_.unlock();
        }

//
//        int num_obst = msg.getCells().size();
//        me.obstacle_lock_.lock();
//        try{
//            me.obstacle_points_.clear();
//
//            for (int i = 0; i < num_obst; i++) {
//                PointStamped in=messageFactory.newFromType(PointStamped._TYPE);
//                PointStamped result=messageFactory.newFromType(PointStamped._TYPE);
//                in.setHeader(msg.getHeader());
//                in.setPoint(msg.getCells().get(i));
//                //ROS_DEBUG("obstacle at %f %f",msg->cells[i].x,msg->cells[i].y);
//                //TODO:Need Transform
///*                try {
//                    tf_->waitForTransform(global_frame_, robot_base_frame_, msg->header.stamp, ros::Duration(0.2));
//
//                    tf_->transformPoint(global_frame_, in, result);
//                }
//                catch (tf::TransformException ex){
//                    ROS_ERROR("%s",ex.what());
//                    return;
//                };*/
//
//                me.obstacle_points_.add(new Vector2(result.getPoint().getX(),result.getPoint().getY()));
//            }
//        }finally {
//            me.obstacle_lock_.unlock();
//        }

    }

    //implement interface setPlan
    public boolean setPlan(final List<PoseStamped> orig_global_plan) {
        if (!initialized_) {
            this.node.getLog().error("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //reset the global plan
        global_plan_ = new ArrayList<PoseStamped>();
        global_plan_ = orig_global_plan;
        current_waypoint_ = 0;
        xy_tolerance_latch_ = false;
        //get the global plan in our frame
        if (!transformGlobalPlan(true, global_plan_, global_frame_, base_frame_, transformed_plan_)) {
            this.node.getLog().warn("Could not transform the global plan to the frame of the controller");
            return false;
        }

//        me.msgPublisher.publishPlan(global_plan_,base_frame_, g_plan_pub_);
//        me.msgPublisher.publishPlan(transformed_plan_,base_frame_, l_plan_pub_);
        return true;
    }

    //implement interface isGoalReached
    public boolean isGoalReached() {
        if (!initialized_) {
            this.node.getLog().error("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //copy over the odometry information
        Odometry base_odom = messageFactory.newFromType(Odometry._TYPE);
        me.me_lock_.lock();
        try {
            base_odom = me.getBaseOdom();
        } finally {
            me.me_lock_.unlock();
        }

        //need implementation map, transform and check goal reached or not
/*        Stamped<Pose> global_pose;
        costmap_ros_->getRobotPose(global_pose);

        costmap_2d::Costmap2D costmap_; ///< @brief The costmap the controller will u
        costmap_ros_->getCostmapCopy(costmap_);


        return base_local_planner::isGoalReached(*tf_,
                global_plan_,
                costmap_,
                global_frame_,
                global_pose,
                base_odom,
                rot_stopped_velocity_,
                trans_stopped_velocity_,
                xy_goal_tolerance_, yaw_goal_tolerance_);*/
        return false;
    }


    //implement interface for computeVelocityCommands
    public boolean computeVelocityCommands(Twist cmd_vel) {
        if (!initialized_) {
            this.node.getLog().error("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //get position and velocity,pose in global frame, velocity is in base frame
        PoseStamped global_pose = node.getTopicMessageFactory().newFromType(PoseStamped._TYPE);
        global_pose.getHeader().setFrameId(global_frame_);
        global_pose.getHeader().setStamp(node.getCurrentTime());
        Twist base_vel = messageFactory.newFromType(Twist._TYPE);

        me.me_lock_.lock();
        try {
            base_vel.setLinear(me.getBaseOdom().getTwist().getTwist().getLinear());//base frame
            base_vel.setAngular(me.getBaseOdom().getTwist().getTwist().getAngular());
            global_pose.setPose(me.getBaseOdom().getPose().getPose());//odometry or map frame
        } finally {
            me.me_lock_.unlock();
        }

        PoseStamped goal_point;
        goal_point = global_plan_.get(global_plan_.size() - 1);
        double goal_x = goal_point.getPose().getPosition().getX();
        double goal_y = goal_point.getPose().getPosition().getY();
        double goal_th = LPutils.getYaw(goal_point.getPose().getOrientation());

        //check to see if we've reached the goal position
        if (xy_tolerance_latch_ || (LPutils.getGoalPositionDistance(global_pose.getPose(), goal_x, goal_y) <= xy_goal_tolerance_)) {

            //if(base_local_planner::goalPositionReached(global_pose, goal_x, goal_y, xy_goal_tolerance_) || xy_tolerance_latch_){

            //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
            //just rotate in place
            if (latch_xy_goal_tolerance_)
                xy_tolerance_latch_ = true;

            //check to see if the goal orientation has been reached
            double angle = LPutils.getGoalOrientationAngleDifference(global_pose.getPose(), goal_th);

            //check to see if the goal orientation has been reached
            if (Math.abs(angle) <= yaw_goal_tolerance_) {

                //set the velocity command to zero
                linear.setX(0.0);
                linear.setY(0.0);
                linear.setZ(0.0);
                cmd_vel.setLinear(linear);
                angular.setZ(0.0);
                cmd_vel.setAngular(angular);
                rotating_to_goal_ = false;
                xy_tolerance_latch_ = false;
            } else {
                //copy over the odometry information
                Odometry base_odom = messageFactory.newFromType(Odometry._TYPE);
                me.me_lock_.lock();
                try {
                    base_odom = me.getBaseOdom();
                } finally {
                    me.me_lock_.unlock();
                }

                //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
                if (!rotating_to_goal_ && !LPutils.stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)) {
                    //ROS_DEBUG("Not stopped yet. base_odom: x=%6.4f,y=%6.4f,z=%6.4f", base_odom.twist.twist.linear.x,base_odom.twist.twist.linear.y,base_odom.twist.twist.angular.z);
                    if (!stopWithAccLimits(global_pose, base_vel, cmd_vel))
                        return false;
                }
                //if we're stopped... then we want to rotate to goal
                else {
                    //set this so that we know its OK to be moving
                    rotating_to_goal_ = true;
                    if (!rotateToGoal(global_pose, base_vel, goal_th, cmd_vel))
                        return false;
                }
            }

            //publish an empty plan because we've reached our goal position
            transformed_plan_.clear();
//            me.msgPublisher.publishPlan(transformed_plan_, id, g_plan_pub_);
//            me.msgPublisher.publishPlan(transformed_plan_, id, l_plan_pub_);
            //we don't actually want to run the controller when we're just rotating to goal
            return true;
        }

        //TODO:transform
/*        tf::Stamped<tf::Pose> target_pose;
        target_pose.setIdentity();
        target_pose.frame_id_ = robot_base_frame_;*/
        PoseStamped target_pose = messageFactory.newFromType(PoseStamped._TYPE);

        if (!skip_next_) {
            if (!transformGlobalPlan(false, global_plan_, global_frame_, base_frame_, transformed_plan_)) {
                this.node.getLog().warn("Could not transform the global plan to the frame of the controller");
                return false;
            }

            findBestWaypoint(target_pose, global_pose);
        }
        //TODO
        /*tf::poseStampedMsgToTF(transformed_plan_[current_waypoint_], target_pose);*/


        Twist pref_vel_twist = messageFactory.newFromType(Twist._TYPE);

        pref_vel_twist.getLinear().setX(target_pose.getPose().getPosition().getX() - global_pose.getPose().getPosition().getX());
        pref_vel_twist.getLinear().setY(target_pose.getPose().getPosition().getY() - global_pose.getPose().getPosition().getY());
//        pref_vel_twist.getAngular().setZ(Angles.shortest_angular_distance(LPutils.getYaw(global_pose.getPose().getOrientation()),
//                Math.atan2(pref_vel_twist.getLinear().getY(), pref_vel_twist.getLinear().getX())));//not used

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

        boolean valid_cmd = true; //collision_planner_.checkTrajectory(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,true);

        if (!valid_cmd) {
            cmd_vel.getAngular().setZ(0);
            cmd_vel.getLinear().setX(0);
            cmd_vel.getLinear().setY(0);
        }

        if (cmd_vel.getLinear().getX() == 0.0 && cmd_vel.getAngular().getZ() == 0.0 && cmd_vel.getLinear().getY() == 0.0) {

            this.node.getLog().debug("Did not find a good vel, calculated best holonomic velocity was:"
                    + me.velocity.getX() + ", " + me.velocity.getY() + ", cur wp " + current_waypoint_ + " of " + transformed_plan_.size() + " trying next waypoint");
            if (current_waypoint_ < transformed_plan_.size() - 1) {
                current_waypoint_++;
                skip_next_ = true;
            } else {
                //reached goal
                transformed_plan_.clear();
//                me.msgPublisher.publishPlan(transformed_plan_,id, g_plan_pub_);
//                me.msgPublisher.publishPlan( transformed_plan_,id, l_plan_pub_);
                return false;
            }
        } else {
            skip_next_ = false;
        }

        List<PoseStamped> local_plan = new ArrayList<PoseStamped>();
        PoseStamped pos = messageFactory.newFromType(PoseStamped._TYPE);

        //TODO
        //tf::poseStampedTFToMsg(global_pose,pos);
        pos.setPose(global_pose.getPose());//global pose is in base frame???
        pos.setHeader(global_pose.getHeader());
        local_plan.add(pos);
        local_plan.add(transformed_plan_.get(current_waypoint_));
//        me.msgPublisher.publishPlan(transformed_plan_,id, g_plan_pub_);
//        me.msgPublisher.publishPlan( local_plan,id, l_plan_pub_);

        return true;
    }


    boolean stopWithAccLimits(PoseStamped global_pose, final Twist robot_vel, Twist cmd_vel) {
        //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
        //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
        double vx = sign(robot_vel.getLinear().getX()) * Math.max(0.0, (Math.abs(robot_vel.getLinear().getX()) - me.acc_lim_x_ * me.simPeriod));
        double vy = sign(robot_vel.getLinear().getY()) * Math.max(0.0, (Math.abs(robot_vel.getLinear().getY()) - me.acc_lim_y_ * me.simPeriod));

        double vth = sign(robot_vel.getAngular().getZ()) * Math.max(0.0, (Math.abs(robot_vel.getAngular().getZ()) - me.acc_lim_th_ * me.simPeriod));

        //we do want to check whether or not the command is valid?????????????
        boolean valid_cmd = true; //collision_planner_.checkTrajectory(vx, vy, vth, true);

        //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
        if (valid_cmd) {
            //ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
            cmd_vel.getLinear().setX(vx);
            cmd_vel.getLinear().setY(vy);
            cmd_vel.getAngular().setZ(vth);
            return true;
        }

        cmd_vel.getLinear().setX(0);
        cmd_vel.getLinear().setY(0);
        cmd_vel.getAngular().setZ(0);
        return false;
    }


    boolean rotateToGoal(final PoseStamped global_pose, final Twist robot_vel, double goal_th, Twist cmd_vel) {
        if (ignore_goal_yaw_) {
            cmd_vel.getAngular().setZ(0);
            return true;
        }
        double yaw = LPutils.getYaw(global_pose.getPose().getOrientation());
        double vel_yaw = robot_vel.getAngular().getZ();
        cmd_vel.getLinear().setX(0);
        cmd_vel.getLinear().setY(0);

        double ang_diff = Angles.shortest_angular_distance(yaw, goal_th);

        double v_th_samp = ang_diff > 0.0 ?
                Math.min(me.max_vel_th_, Math.max(me.min_vel_th_inplace_, ang_diff)) :
                Math.max(-1.0 * me.max_vel_th_, Math.min(-1.0 * me.min_vel_th_inplace_, ang_diff));

        //take the acceleration limits of the robot into account
        double max_acc_vel = Math.abs(vel_yaw) + me.acc_lim_th_ * me.simPeriod;
        double min_acc_vel = Math.abs(vel_yaw) - me.acc_lim_th_ * me.simPeriod;

        v_th_samp = sign(v_th_samp) * Math.min(Math.max(Math.abs(v_th_samp), min_acc_vel), max_acc_vel);

        //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
        double max_speed_to_stop = Math.sqrt(2 * me.acc_lim_th_ * Math.abs(ang_diff));//how to get this???

        v_th_samp = sign(v_th_samp) * Math.min(max_speed_to_stop, Math.abs(v_th_samp));
        if (Math.abs(v_th_samp) <= 0.0 * me.min_vel_th_inplace_)//???????????
            v_th_samp = 0.0;
        else if (Math.abs(v_th_samp) < me.min_vel_th_inplace_)
            v_th_samp = sign(v_th_samp) * Math.max(me.min_vel_th_inplace_, Math.abs(v_th_samp));
        //we still want to lay down the footprint of the robot and check if the action is legal
        boolean valid_cmd = true;//collision_planner_.checkTrajectory(0.0, 0.0, v_th_samp,true);

        this.node.getLog().debug("Moving to desired goal orientation, th cmd: %1$2f" + v_th_samp);

        if (valid_cmd) {
            cmd_vel.getAngular().setZ(v_th_samp);
            return true;
        }
        cmd_vel.getAngular().setZ(0);
        return false;
    }

    // transform global plan
    boolean transformGlobalPlan(boolean initialPlan, final List<PoseStamped> global_plan, final String global_frame, final String base_frame, List<PoseStamped> transformed_plan) {

        final PoseStamped plan_pose = messageFactory.newFromType(PoseStamped._TYPE);

        plan_pose.setPose(global_plan.get(0).getPose());
        plan_pose.setHeader(global_plan.get(0).getHeader());

        transformed_plan.clear();

        //TODO:exception handle

        if (!(global_plan.size() > 0)) {
            this.node.getLog().error("Recieved plan with zero length");
            return false;
        }

        //currently global plan is in global frame do not need transform
        //TODO: transform
/*
            tf.lookupTransform(global_frame, ros::Time(),
                    plan_pose.header.frame_id, plan_pose.header.stamp,
                    plan_pose.header.frame_id, transform);
*/
        Time t;
        Pose robot_pose = node.getTopicMessageFactory().newFromType(Pose._TYPE);
        int cur_waypoint = 0;
        if (!initialPlan) {

            me.me_lock_.lock();
            try {
                robot_pose.setPosition(me.getBaseOdom().getPose().getPose().getPosition());
                if (me.getLastSeen() == null)
                    t = node.getCurrentTime();
                else
                    t = me.getLastSeen();
            } finally {
                me.me_lock_.unlock();
            }
//            tf3d=tf.transform().lookupTransformBetween(global_frame,plan_pose.getHeader().getFrameId(),plan_pose.getHeader().getStamp().totalNsecs());
//            StampedTransform transform=new StampedTransform(plan_pose.getHeader().getStamp().totalNsecs(),
//                    transf.parentFrame,transf.childFrame,transf.translation,transf.rotation);

            //TODO: transform
            //let's get the pose of the robot in the frame of the plan
            //PoseStamped robot_pose=messageFactory.newFromType(PoseStamped._TYPE);
/*            robot_pose.setIdentity();
            robot_pose.frame_id_ = costmap.getBaseFrameID();
            robot_pose.stamp_ = ros::Time();
            tf.transformPose(plan_pose.header.frame_id, robot_pose, robot_pose);
*/

            //we'll keep points on the plan that are within the window that we're looking at

            double sq_dist = Double.MAX_VALUE;

            double dist;

            for (int i = 0; i < global_plan_.size(); i++) {
                dist = LPutils.getGoalPositionDistance(
                        robot_pose,
                        global_plan_.get(i).getPose().getPosition().getX(),
                        global_plan_.get(i).getPose().getPosition().getY());

                if (dist < Math.sqrt(sq_dist)) {
                    sq_dist = dist * dist;
                    cur_waypoint = i;
                }
            }

        } else {
            t = node.getCurrentTime();
        }

        PoseStamped tf_pose;
        PoseStamped newer_pose;

        int i = cur_waypoint;

        //now we'll transform until points are outside of our distance threshold
        while (i < global_plan.size()) {
            //double x_diff = robot_pose.getPose().getPosition().getX() - global_plan.get(i).getPose().getPosition().getX();
            //double y_diff = robot_pose.getPose().getPosition().getY() - global_plan.get(i).getPose().getPosition().getY();
            //sq_dist = x_diff * x_diff + y_diff * y_diff;

            PoseStamped pose = node.getTopicMessageFactory().newFromType(PoseStamped._TYPE);
            pose.setHeader(global_plan.get(i).getHeader());
            pose.getHeader().setStamp(t);
            pose.setPose(global_plan.get(i).getPose());

/*                poseStampedMsgToTF(pose, tf_pose);
                tf_pose.setData(transform * tf_pose);
                tf_pose.stamp_ = transform.stamp_;
                tf_pose.frame_id_ = global_frame;
                poseStampedTFToMsg(tf_pose, newer_pose);
 */
            transformed_plan.add(pose);

            ++i;
        }

/*        }
        catch(tf::LookupException& ex) {
            this.node.getLog().error("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch(tf::ConnectivityException& ex) {
            this.node.getLog().error("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch(tf::ExtrapolationException& ex) {
            this.node.getLog().error("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                this.node.getLog().error("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsignedint)global_plan.size(), global_plan[0].header.frame_id.c_str());

            return false;
        }*/

        return true;
    }


    void findBestWaypoint(PoseStamped target_pose, final PoseStamped global_pose) {
        current_waypoint_ = 0;
        double min_dist = Double.MAX_VALUE;
        for (int i = current_waypoint_; i < transformed_plan_.size(); i++) {
            double dist = LPutils.getGoalPositionDistance(global_pose.getPose(), transformed_plan_.get(i).getPose().getPosition().getX(),
                    transformed_plan_.get(i).getPose().getPosition().getY());
            if (dist < me.getRadius() || dist < min_dist) {
                min_dist = dist;
                target_pose.setHeader(transformed_plan_.get(i).getHeader());
                target_pose.setPose(transformed_plan_.get(i).getPose());
                current_waypoint_ = i;

            }
        }
        //ROS_DEBUG("waypoint = %d, of %d", current_waypoint_, transformed_plan_.size());

        if (current_waypoint_ == transformed_plan_.size() - 1) //I am at the end of the plan
            return;

        double dif_x = transformed_plan_.get(current_waypoint_ + 1).getPose().getPosition().getX() - target_pose.getPose().getPosition().getX();
        double dif_y = transformed_plan_.get(current_waypoint_ + 1).getPose().getPosition().getY() - target_pose.getPose().getPosition().getY();

        double plan_dir = Math.atan2(dif_y, dif_x);
        double dif_ang = plan_dir;

        //ROS_DEBUG("dif = %f,%f of %f",dif_x,dif_y, dif_ang );


        for (int i = current_waypoint_ + 1; i < transformed_plan_.size(); i++) {
            dif_x = transformed_plan_.get(i).getPose().getPosition().getX() - target_pose.getPose().getPosition().getX();
            dif_y = transformed_plan_.get(i).getPose().getPosition().getY() - target_pose.getPose().getPosition().getY();

            dif_ang = Math.atan2(dif_y, dif_x);

            if (Math.abs(plan_dir - dif_ang) > 1.0 * yaw_goal_tolerance_) {
                target_pose.setHeader(transformed_plan_.get(i - 1).getHeader());
                target_pose.setPose(transformed_plan_.get(i - 1).getPose());
                current_waypoint_ = i - 1;
                //ROS_DEBUG("waypoint = %d, of %d", current_waypoint_, transformed_plan_.size());

                return;
            }
        }
        target_pose.setHeader(transformed_plan_.get(transformed_plan_.size() - 1).getHeader());
        target_pose.setPose(transformed_plan_.get(transformed_plan_.size() - 1).getPose());
        current_waypoint_ = transformed_plan_.size() - 1;
    }

}
