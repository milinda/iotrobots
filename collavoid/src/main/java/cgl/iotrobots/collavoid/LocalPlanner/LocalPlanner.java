package cgl.iotrobots.collavoid.LocalPlanner;

import cgl.iotrobots.collavoid.ROSAgent.Agent;
import cgl.iotrobots.collavoid.utils.Angles;
import cgl.iotrobots.collavoid.utils.Vector2;
import costmap_2d.VoxelGrid;
import geometry_msgs.*;
import nav_msgs.GridCells;
import nav_msgs.Odometry;
import nav_msgs.Path;
import org.apache.commons.logging.Log;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.*;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.*;
import org.ros.rosjava.tf.pubsub.TransformListener;

import java.lang.String;
import java.net.InetAddress;
import java.net.URI;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

import static cgl.iotrobots.collavoid.utils.utils.sign;


public class LocalPlanner extends AbstractNodeMain{
    //currently no dynamic reconfigurer

    //rosjava node params
    String name;
    protected ConnectedNode node;
    private ParameterTree params;

    //costmap Datatypes:
    VoxelGrid costmap_ros_;
    VoxelGrid costmap_; ///< @brief The costmap the controller will use

    TransformListener tf_;

    boolean initialized_, skip_next_, setup_;

    //Agent stuff
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
    double  time_horizon_obst_;
    double eps_;

    Agent me;

    double time_to_holo_, min_error_holo_, max_error_holo_;

    String global_frame_; ///< @brief The frame in which the controller will run
    String robot_base_frame_; ///< @brief Used as the base frame id of the robot
    List<PoseStamped> global_plan_, transformed_plan_;
    Publisher g_plan_pub_, l_plan_pub_;
    Subscriber obstacles_sub_;
    String hostname;

    private static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    private static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);
    private static Vector3 linear=messageFactory.newFromType(Vector3._TYPE);
    private static Vector3 angular=messageFactory.newFromType(Vector3._TYPE);//for temporary assign use
    private static Logger logger;


    /*---------------methods begin-----------------*/
    //must pass connectednode,tf will be initialized from the caller, see rosjava_tf TfViz
    public LocalPlanner(String hostname,TransformListener tf, VoxelGrid costmap_ros){
        this.hostname=new String(hostname);
        tf_=tf;
        initialized_=false;
        costmap_ros_ = costmap_ros;
        current_waypoint_ = 0;
        skip_next_ = false;
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        this.node=connectedNode;
        this.params=this.node.getParameterTree();
        initialize();
    }

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }


    //implement the interface of nav_core::BaseLocalPlanner Class in original ROS
    private void initialize(){
        if (!initialized_){
            logger=Logger.getLogger(node.getName().toString());
            rot_stopped_velocity_ = params.getDouble("~/rot_stopped_velocity",0.01);
            trans_stopped_velocity_ = params.getDouble("~/trans_stopped_velocity_",0.01);

            //base local planner params
            yaw_goal_tolerance_ = params.getDouble("~/yaw_goal_tolerance", 0.05);
            xy_goal_tolerance_  = params.getDouble("~/xy_goal_tolerance", 0.10);
            latch_xy_goal_tolerance_ = params.getBoolean("~/latch_xy_goal_tolerance", false);
            ignore_goal_yaw_ = params.getBoolean("~/ignore_goal_jaw", false);

            /*----------spawn agent node---------*/
            spawnAgent(hostname,tf_);
            node.getLog().info("************************Spawning the agent.......");
            while(!me.initialized_){
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            radius_=me.footprint_radius_;

            Publisher<Path> g_plan_pub_=this.node.newPublisher("global_plan", Path._TYPE);
            Publisher<Path> l_plan_pub_=this.node.newPublisher("l_plan_pub_", Path._TYPE);
            String move_base_name = this.node.getName().toString();
            //ROS_ERROR("%s name of node", thisname.c_str());

            Subscriber<GridCells> obstacles_sub_ = this.node.newSubscriber(move_base_name + "/local_costmap/obstacles", GridCells._TYPE);
            obstacles_sub_.addMessageListener(new MessageListener<GridCells>() {
                @Override
                public void onNewMessage(GridCells msg) {
                    obstaclesCallback(msg);
                }
            });

            //setup_= false;??????????????
            //not implemented yet
            //dsrv_ = new dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>(private_nh);
            //dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>::CallbackType cb = boost::bind(&CollvoidLocalPlanner::reconfigureCB, this, _1, _2);
            //dsrv_->setCallback(cb);

            initialized_ = true;
            node.getLog().info("************************"+node.getName()+" is initialized!");
        }
        else {
            node.getLog().info("************************"+node.getName()+" has already been initialized, you can't call it twice, doing nothing");
        }

    }
    /*Spawn agent*/
    void spawnAgent(String id,TransformListener tf){
        NodeConfiguration configuration = NodeConfiguration.newPublic(node.getUri().getHost(),
                node.getMasterUri());

        final NodeMainExecutor runner= DefaultNodeMainExecutor.newDefault();
        String agentId=new String("agent_"+id);
        this.me=new Agent(agentId,tf);
        configuration.setNodeName(agentId);
        runner.execute(me,configuration);
    }

    void obstaclesCallback(final GridCells msg){
        int num_obst = msg.getCells().size();
        me.obstacle_lock_.lock();
        try{
            me.obstacle_points_.clear();

            for (int i = 0; i < num_obst; i++) {
                PointStamped in=messageFactory.newFromType(PointStamped._TYPE);
                PointStamped result=messageFactory.newFromType(PointStamped._TYPE);
                in.setHeader(msg.getHeader());
                in.setPoint(msg.getCells().get(i));
                //ROS_DEBUG("obstacle at %f %f",msg->cells[i].x,msg->cells[i].y);
                //TODO:Need Transform
/*                try {
                    tf_->waitForTransform(global_frame_, robot_base_frame_, msg->header.stamp, ros::Duration(0.2));

                    tf_->transformPoint(global_frame_, in, result);
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    return;
                };*/

                me.obstacle_points_.add(new Vector2(result.getPoint().getX(),result.getPoint().getY()));
            }
        }finally {
            me.obstacle_lock_.unlock();
        }

    }

    //implement interface setPlan
    boolean setPlan(final List<PoseStamped> orig_global_plan){
        if(!initialized_){
            this.node.getLog().error("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //reset the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        current_waypoint_ = 0;
        xy_tolerance_latch_ = false;
        //transformed_plan = global_plan;
        //get the global plan in our frame
        if(!transformGlobalPlan(tf_, global_plan_, costmap_ros_, global_frame_, transformed_plan_)){
            this.node.getLog().warn("Could not transform the global plan to the frame of the controller");
            return false;
        }

        return true;
    }
    //implement interface isGoalReached
    public boolean isGoalReached(){
        if(!initialized_){
            this.node.getLog().error("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //copy over the odometry information
        Odometry base_odom=messageFactory.newFromType(Odometry._TYPE);
            me.me_lock_.lock();
            try{
                base_odom = me.base_odom_;
            }finally {
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
    boolean computeVelocityCommands(Twist cmd_vel){
        if(!initialized_){
            this.node.getLog().error("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //TODO: Transform
/*        PoseStamped global_pose;
        Header hdr=messageFactory.newFromType(Header._TYPE);
        hdr.setStamp(this.node.getCurrentTime());
        hdr.setFrameId(robot_base_frame_);
        global_pose.setHeader(hdr);
        Pose pose=messageFactory.newFromType(Pose._TYPE);
        Point p=messageFactory.newFromType(Point._TYPE);
        Quaternion q=messageFactory.newFromType(Quaternion._TYPE);
        p.setX(0.0);p.setY(0.0);p.setZ(0.0);
        pose.setPosition(p);
        q.setX(0.0);q.setY(0.0);q.setZ(0.0);q.setW(1);
        pose.setOrientation(q);
        global_pose.setPose(pose);
        tf_.getTree().lookupTransformBetween();*/



        // Set current velocities from odometry
        Twist global_vel=messageFactory.newFromType(Twist._TYPE);

        me.me_lock_.lock();
        try{
            linear.setX(me.base_odom_.getTwist().getTwist().getLinear().getX());
            linear.setY(me.base_odom_.getTwist().getTwist().getLinear().getY());
            angular.setZ(me.base_odom_.getTwist().getTwist().getAngular().getY());
            global_vel.setLinear(linear);
            global_vel.setAngular(angular);
        }finally {
            me.me_lock_.unlock();
        }

        //TODO: Transform
        //WARNNING not transformed
/*        tf::Stamped<tf::Pose> robot_vel;
        robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
        robot_vel.frame_id_ = robot_base_frame_;
        robot_vel.stamp_ = ros::Time();*/
        PoseStamped robot_vel=messageFactory.newFromType(PoseStamped._TYPE);

/*        //WARNNING not transformed
        tf::Stamped<tf::Pose> goal_point;
        tf::poseStampedMsgToTF(global_plan_.back(), goal_point);*/
        PoseStamped goal_point=messageFactory.newFromType(PoseStamped._TYPE);

        //we assume the global goal is the last point in the global plan
        //WARNNING not transformed
        goal_point=global_plan_.get(global_plan_.size()-1);
        double goal_x = goal_point.getPose().getPosition().getX();
        double goal_y = goal_point.getPose().getPosition().getY();
        double yaw = LPutils.getYaw(goal_point.getPose().getOrientation());
        double goal_th = yaw;

        //TODO:get position in global frame of the robot
        //WARNNING not transformed
        /*        tf::Stamped<tf::Pose> global_pose;
        //let's get the pose of the robot in the frame of the plan
        global_pose.setIdentity();
        global_pose.frame_id_ = robot_base_frame_;
        global_pose.stamp_ = ros::Time();
        tf_->transformPose(global_frame_, global_pose, global_pose);*/
        PoseStamped global_pose=messageFactory.newFromType(PoseStamped._TYPE);

        //check to see if we've reached the goal position
        if (xy_tolerance_latch_ || (LPutils.getGoalPositionDistance(global_pose.getPose(), goal_x, goal_y) <= xy_goal_tolerance_)) {

            //if(base_local_planner::goalPositionReached(global_pose, goal_x, goal_y, xy_goal_tolerance_) || xy_tolerance_latch_){

            //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
            //just rotate in place
            if(latch_xy_goal_tolerance_)
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
            }
            else {
                //copy over the odometry information
                Odometry base_odom=messageFactory.newFromType(Odometry._TYPE);
                me.me_lock_.lock();
                try{
                    base_odom=me.getBaseOdom();
                }finally {
                    me.me_lock_.unlock();
                }

                //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
                if(!rotating_to_goal_ && !LPutils.stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)){
                    //ROS_DEBUG("Not stopped yet. base_odom: x=%6.4f,y=%6.4f,z=%6.4f", base_odom.twist.twist.linear.x,base_odom.twist.twist.linear.y,base_odom.twist.twist.angular.z);
                    if(!stopWithAccLimits(global_pose, robot_vel, cmd_vel))
                        return false;
                }
                //if we're stopped... then we want to rotate to goal
                else{
                    //set this so that we know its OK to be moving
                    rotating_to_goal_ = true;
                    if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel))
                        return false;
                }
            }

            //publish an empty plan because we've reached our goal position
            transformed_plan_.clear();
            LPutils.publishPlan(transformed_plan_, g_plan_pub_);
            LPutils.publishPlan(transformed_plan_, l_plan_pub_);
            //we don't actually want to run the controller when we're just rotating to goal
            return true;
        }

        //TODO:transform
/*        tf::Stamped<tf::Pose> target_pose;
        target_pose.setIdentity();
        target_pose.frame_id_ = robot_base_frame_;*/
        PoseStamped target_pose=messageFactory.newFromType(PoseStamped._TYPE);

        if (!skip_next_){
            if(!transformGlobalPlan(tf_, global_plan_, costmap_ros_, global_frame_, transformed_plan_)){
                this.node.getLog().warn("Could not transform the global plan to the frame of the controller");
                return false;
            }
            PoseStamped target_pose_msg=messageFactory.newFromType(PoseStamped._TYPE);
            findBestWaypoint(target_pose_msg, global_pose);
        }
        //TODO
        /*tf::poseStampedMsgToTF(transformed_plan_[current_waypoint_], target_pose);*/


        Twist res=messageFactory.newFromType(Twist._TYPE);

        linear.setX(target_pose.getPose().getPosition().getX() - global_pose.getPose().getPosition().getX());
        linear.setY(target_pose.getPose().getPosition().getY() - global_pose.getPose().getPosition().getY());
        res.setLinear(linear);
        angular.setZ(Angles.shortest_angular_distance(LPutils.getYaw(global_pose.getPose().getOrientation()), Math.atan2(res.getLinear().getY(), res.getLinear().getX())));
        res.setAngular(angular);

        Vector2 goal_dir = new Vector2(res.getLinear().getX(),res.getLinear().getY());
        if (Vector2.abs(goal_dir) > me.max_vel_x_) {
            goal_dir = Vector2.mul(Vector2.normalize(goal_dir),me.max_vel_x_);
        }
        else if (Vector2.abs(goal_dir) < me.min_vel_x_) {
            goal_dir =Vector2.mul(Vector2.normalize(goal_dir), me.min_vel_x_ * 1.2);
        }


        Vector2 pref_vel = new Vector2(goal_dir.getX(),goal_dir.getY());

        //TODO collvoid added

        me.computeNewVelocity(pref_vel, cmd_vel);


        if(Math.abs(cmd_vel.getAngular().getZ())<me.min_vel_th_)
            angular.setZ(0.0);

        if(Math.abs(cmd_vel.getLinear().getX())<me.min_vel_x_)
            linear.setX(0.0);

        if(Math.abs(cmd_vel.getLinear().getY())<me.min_vel_y_)
            linear.setY(0.0);

        cmd_vel.setLinear(linear);
        cmd_vel.setAngular(angular);


        boolean valid_cmd = true; //collision_planner_.checkTrajectory(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,true);

        if (!valid_cmd){
            angular.setZ(0.0);
            linear.setX(0.0);
            linear.setY(0.0);
            cmd_vel.setLinear(linear);
            cmd_vel.setAngular(angular);
        }

        if (cmd_vel.getLinear().getX() == 0.0 && cmd_vel.getAngular().getZ() == 0.0 && cmd_vel.getLinear().getY() == 0.0) {

            this.node.getLog().debug("Did not find a good vel, calculated best holonomic velocity was:"
                    + me.velocity.getX()+", "+me.velocity.getY()+", cur wp "+current_waypoint_+" of "+transformed_plan_.size()+" trying next waypoint");
            if (current_waypoint_ < transformed_plan_.size()-1){
                current_waypoint_++;
                skip_next_= true;
            }
            else {
                transformed_plan_.clear();
                LPutils.publishPlan(transformed_plan_, g_plan_pub_);
                LPutils.publishPlan(transformed_plan_, l_plan_pub_);

                return false;
            }
        }
        else {
            skip_next_ = false;
        }


        List<PoseStamped> local_plan=new ArrayList<PoseStamped>();
        PoseStamped pos=messageFactory.newFromType(PoseStamped._TYPE);

        //TODO
        //tf::poseStampedTFToMsg(global_pose,pos);
        pos.setPose(global_pose.getPose());
        pos.setHeader(global_pose.getHeader());
        local_plan.add(pos);
        local_plan.add(transformed_plan_.get(current_waypoint_));
        LPutils.publishPlan(transformed_plan_, g_plan_pub_);
        LPutils.publishPlan(local_plan, l_plan_pub_);
        //me_->publishOrcaLines();
        return true;
    }


    boolean stopWithAccLimits(PoseStamped global_pose, final PoseStamped  robot_vel, Twist cmd_vel){
        //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
        //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
        double vx = sign(robot_vel.getPose().getPosition().getX()) * Math.max(0.0, (Math.abs(robot_vel.getPose().getPosition().getX()) - me.acc_lim_x_ * me.simPeriod));
        double vy = sign(robot_vel.getPose().getPosition().getY()) * Math.max(0.0, (Math.abs(robot_vel.getPose().getPosition().getY()) - me.acc_lim_y_ * me.simPeriod));

        double vel_yaw = LPutils.getYaw(robot_vel.getPose().getOrientation());
        double vth = sign(vel_yaw) * Math.max(0.0, (Math.abs(vel_yaw) - me.acc_lim_th_ * me.simPeriod));

        //we do want to check whether or not the command is valid?????????????
        boolean valid_cmd = true; //collision_planner_.checkTrajectory(vx, vy, vth, true);

        //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
        if(valid_cmd){
            //ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
            linear.setX(vx);
            linear.setY(vy);
            cmd_vel.setLinear(linear);
            angular.setZ(vth);
            cmd_vel.setAngular(angular);
            return true;
        }

        linear.setX(0.0);
        linear.setY(0.0);
        linear.setZ(0.0);
        cmd_vel.setLinear(linear);
        return false;
    }


    boolean rotateToGoal(final PoseStamped global_pose, final PoseStamped robot_vel, double goal_th, Twist cmd_vel){
        if (ignore_goal_yaw_) {
            angular.setZ(0.0);
            cmd_vel.setAngular(angular);
            return true;
        }
        double yaw = LPutils.getYaw(global_pose.getPose().getOrientation());
        double vel_yaw = LPutils.getYaw(robot_vel.getPose().getOrientation());
        linear.setX(0.0);
        linear.setY(0.0);
        cmd_vel.setLinear(linear);

        double ang_diff = Angles.shortest_angular_distance(yaw, goal_th);

        double v_th_samp = ang_diff > 0.0 ? Math.min(me.max_vel_th_,
                Math.max(me.min_vel_th_inplace_, ang_diff)) : Math.max(-1.0 * me.max_vel_th_,
                Math.min(-1.0 * me.min_vel_th_inplace_, ang_diff));

        //take the acceleration limits of the robot into account
        double max_acc_vel = Math.abs(vel_yaw) + me.acc_lim_th_ * me.simPeriod;
        double min_acc_vel = Math.abs(vel_yaw) - me.acc_lim_th_ * me.simPeriod;

        v_th_samp = sign(v_th_samp) * Math.min(Math.max(Math.abs(v_th_samp), min_acc_vel), max_acc_vel);

        //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
        double max_speed_to_stop = Math.sqrt(2 * me.acc_lim_th_ * Math.abs(ang_diff));

        v_th_samp = sign(v_th_samp) * Math.min(max_speed_to_stop, Math.abs(v_th_samp));
        if (Math.abs(v_th_samp) <= 0.0 * me.min_vel_th_inplace_)
            v_th_samp  = 0.0;
        else if (Math.abs(v_th_samp) < me.min_vel_th_inplace_)
            v_th_samp = sign(v_th_samp) * Math.max(me.min_vel_th_inplace_,Math.abs(v_th_samp));
        //we still want to lay down the footprint of the robot and check if the action is legal
        boolean valid_cmd = true;//collision_planner_.checkTrajectory(0.0, 0.0, v_th_samp,true);

        this.node.getLog().debug("Moving to desired goal orientation, th cmd: %1$2f"+v_th_samp);

        if(valid_cmd){
            angular.setZ(v_th_samp);
            cmd_vel.setAngular(angular);
            return true;
        }
        angular.setZ(0.0);
        cmd_vel.setAngular(angular);
        return false;
    }

    boolean transformGlobalPlan(final TransformListener tf,final List<PoseStamped> global_plan,final VoxelGrid costmap,final String global_frame,
            List<PoseStamped> transformed_plan){

        final PoseStamped plan_pose = messageFactory.newFromType(PolygonStamped._TYPE);

        plan_pose.setPose(global_plan.get(0).getPose());
        plan_pose.setHeader(global_plan.get(0).getHeader());

        transformed_plan.clear();

        //TODO:exception handle
//        try{
            if (!(global_plan.size() > 0))
            {
                this.node.getLog().error("Recieved plan with zero length");
                return false;
            }

            //TODO: transform
/*
            tf.lookupTransform(global_frame, ros::Time(),
                    plan_pose.header.frame_id, plan_pose.header.stamp,
                    plan_pose.header.frame_id, transform);
*/
            org.ros.rosjava.tf.Transform transf;
            transf=tf.getTree().lookupTransformBetween(global_frame,plan_pose.getHeader().getFrameId(),plan_pose.getHeader().getStamp().totalNsecs());
            StampedTransform transform=new StampedTransform(plan_pose.getHeader().getStamp().totalNsecs(),
                    transf.parentFrame,transf.childFrame,transf.translation,transf.rotation);

            //TODO: transform
            //let's get the pose of the robot in the frame of the plan
            PoseStamped robot_pose=messageFactory.newFromType(PoseStamped._TYPE);
/*            robot_pose.setIdentity();
            robot_pose.frame_id_ = costmap.getBaseFrameID();
            robot_pose.stamp_ = ros::Time();
            tf.transformPose(plan_pose.header.frame_id, robot_pose, robot_pose);
*/

            //we'll keep points on the plan that are within the window that we're looking at

            double sq_dist = Double.MAX_VALUE;

            int cur_waypoint = 0;
            for (int i=0; i < global_plan_.size(); i++)
            {
                double dist = LPutils.getGoalPositionDistance(
                        robot_pose.getPose(),
                        global_plan_.get(i).getPose().getPosition().getX(),
                        global_plan_.get(i).getPose().getPosition().getY());

                if (dist < Math.sqrt(sq_dist)) {
                    sq_dist = dist * dist;
                    cur_waypoint = i;
                }
            }

            int i = cur_waypoint;

            PoseStamped tf_pose;
            PoseStamped newer_pose;

            //now we'll transform until points are outside of our distance threshold
            while(i < global_plan.size()) {
                double x_diff = robot_pose.getPose().getPosition().getX() - global_plan.get(i).getPose().getPosition().getX();
                double y_diff = robot_pose.getPose().getPosition().getY() - global_plan.get(i).getPose().getPosition().getY();
                sq_dist = x_diff * x_diff + y_diff * y_diff;

                final PoseStamped pose = global_plan.get(i);
/*                poseStampedMsgToTF(pose, tf_pose);
                tf_pose.setData(transform * tf_pose);
                tf_pose.stamp_ = transform.stamp_;
                tf_pose.frame_id_ = global_frame;
                poseStampedTFToMsg(tf_pose, newer_pose);

 */
                transformed_plan.add(global_plan.get(i));

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

    void findBestWaypoint(PoseStamped target_pose, final PoseStamped global_pose){
        current_waypoint_ = 0;
        double min_dist = Double.MAX_VALUE;
        for (int i=current_waypoint_; i < transformed_plan_.size(); i++)
        {
            double dist = LPutils.getGoalPositionDistance(global_pose.getPose(), transformed_plan_.get(i).getPose().getPosition().getX(),
                    transformed_plan_.get(i).getPose().getPosition().getY());
            if (dist < me.radius || dist < min_dist) {
                min_dist = dist;
                target_pose.setHeader(transformed_plan_.get(i).getHeader());
                current_waypoint_ = i;

            }
        }
        //ROS_DEBUG("waypoint = %d, of %d", current_waypoint_, transformed_plan_.size());

        if (current_waypoint_ == transformed_plan_.size()-1) //I am at the end of the plan
            return;

        double dif_x = transformed_plan_.get(current_waypoint_+1).getPose().getPosition().getX() - target_pose.getPose().getPosition().getX();
        double dif_y = transformed_plan_.get(current_waypoint_+1).getPose().getPosition().getY() - target_pose.getPose().getPosition().getY();

        double plan_dir = Math.atan2(dif_y, dif_x);
        double dif_ang = plan_dir;

        //ROS_DEBUG("dif = %f,%f of %f",dif_x,dif_y, dif_ang );


        for (int i=current_waypoint_+1; i<transformed_plan_.size(); i++) {
            dif_x = transformed_plan_.get(i).getPose().getPosition().getX() - target_pose.getPose().getPosition().getX();
            dif_y = transformed_plan_.get(i).getPose().getPosition().getY() - target_pose.getPose().getPosition().getY();

            dif_ang = Math.atan2(dif_y, dif_x);

            if(Math.abs(plan_dir - dif_ang) > 1.0* yaw_goal_tolerance_) {
                target_pose.setHeader(transformed_plan_.get(i-1).getHeader());
                target_pose.setPose(transformed_plan_.get(i-1).getPose());
                current_waypoint_ = i-1;
                //ROS_DEBUG("waypoint = %d, of %d", current_waypoint_, transformed_plan_.size());

                return;
            }
        }
        target_pose.setHeader(transformed_plan_.get(transformed_plan_.size()-1).getHeader());
        target_pose.setPose(transformed_plan_.get(transformed_plan_.size() - 1).getPose());
        current_waypoint_ = transformed_plan_.size()-1;
    }


}
