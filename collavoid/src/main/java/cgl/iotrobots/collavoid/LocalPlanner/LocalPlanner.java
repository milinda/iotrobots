package cgl.iotrobots.collavoid.LocalPlanner;

import cgl.iotrobots.collavoid.ROSAgent.Agent;
import cgl.iotrobots.collavoid.utils.Vector2;
import costmap_2d.VoxelGrid;
import geometry_msgs.*;
import nav_msgs.GridCells;
import nav_msgs.Odometry;
import nav_msgs.Path;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.pubsub.TransformListener;

import java.lang.String;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;


/**
 * Created by hjh on 11/3/14.
 */

public class LocalPlanner{
    //currently no dynamic reconfigurer

    //rosjava node params
    String name;
    protected final ConnectedNode node;
    private ParameterTree params;

    //costmap Datatypes:
    VoxelGrid costmap_ros_;
    VoxelGrid costmap_; ///< @brief The costmap the controller will use

    TransformListener tf_;

    boolean initialized_, skip_next_, setup_;

    //Agent stuff
    double max_vel_x_, min_vel_x_;
    double max_vel_y_, min_vel_y_;
    double max_vel_th_, min_vel_th_, min_vel_th_inplace_;
    double acc_lim_x_, acc_lim_y_, acc_lim_th_;
    double wheel_base_, radius_;
    double max_vel_with_obstacles_;

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

    private static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    private static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);


    /*---------------methods begin-----------------*/
    //must pass connectednode,tf will be initialized from the caller, see rosjava_tf TfViz
    public LocalPlanner(final ConnectedNode node, TransformListener tf, VoxelGrid costmap_ros){
        costmap_ros_=null;
        tf_=null;
        initialized_=false;
        this.node=node;
        //initialize the planner
        initialize(tf, costmap_ros);
    }

    //implement the interface of nav_core::BaseLocalPlanner Class in original ROS
    private void initialize(TransformListener tf, VoxelGrid costmap_ros){
        if (!initialized_){
            tf_ = tf;
            costmap_ros_ = costmap_ros;

            rot_stopped_velocity_ = 0.01;
            trans_stopped_velocity_ = 0.01;

            current_waypoint_ = 0;

            this.params=this.node.getParameterTree();

            //base local planner params
            yaw_goal_tolerance_ = params.getDouble("yaw_goal_tolerance", 0.05);
            xy_goal_tolerance_  = params.getDouble("xy_goal_tolerance", 0.10);
            latch_xy_goal_tolerance_ = params.getBoolean("latch_xy_goal_tolerance", false);
            ignore_goal_yaw_ = params.getBoolean("ignore_goal_jaw", false);


           // ros::NodeHandle nh;//for stand alone??

            String IP = null;
            String hostname = null;
            try {
                InetAddress ia = InetAddress.getLocalHost();
                hostname = ia.getHostName();//获取计算机主机名
                IP = ia.getHostAddress();//获取计算机IP
            } catch (UnknownHostException e) {
                e.printStackTrace();
            }

            //if (standalone_) {
            //    ConnectedNode selfNode;
            String my_id = node.getName().toString();
            if (my_id.equals("/")) {
                my_id = hostname;
            }

            my_id = params.getString("name",my_id);
            node.getLog().info("My name is: " + my_id);


            /*----------init ros agent and set parameters---------*/
            me=new Agent();

            //acceleration limits load from params_turtle.yaml and private_nh
            //namespace maybe /CollvoidLocalPlanneri
            me.acc_lim_x_=params.getDouble("acc_lim_x");
            me.acc_lim_y_=params.getDouble("acc_lim_y");
            me.acc_lim_th_=params.getDouble("acc_lim_th");

            //holo_robot
            me.holo_robot_=params.getBoolean("holo_robot");

            if (!holo_robot_)
                me.wheel_base_=params.getDouble("wheel_base");
            else
                me.wheel_base_ = 0.0;

            //min max speeds
            me.max_vel_with_obstacles_=params.getDouble("max_vel_with_obstacles");

            me.max_vel_x_=params.getDouble("max_vel_x");
            me.min_vel_x_=params.getDouble("min_vel_x");
            me.max_vel_y_=params.getDouble("max_vel_y");
            me.min_vel_y_=params.getDouble("min_vel_y");
            me.max_vel_th_=params.getDouble("max_vel_th");
            me.min_vel_th_=params.getDouble("min_vel_th");
            me.min_vel_th_inplace_=params.getDouble("min_vel_th_inplace");

            //set radius
            me.footprint_radius_=params.getDouble("footprint_radius");
            me.radius=me.footprint_radius_+me.cur_loc_unc_radius_;
            radius_=me.footprint_radius_;

            //set frames
            //robot_base_frame_ = costmap_ros_->getBaseFrameID(); need to investigate
            me.base_frame_ = new String(costmap_ros_.getHeader().getFrameId());//may not be right
            me.global_frame_ = new String(params.getString("global_frame", "/map"));

            //sim period
            GraphName controller_frequency_param_name;
            controller_frequency_param_name=params.search("controller_frequency");
            double sim_period_;
            if(controller_frequency_param_name==null){
                sim_period_ = 0.05;
            }
            else {
                double controller_frequency = 0;
                controller_frequency=params.getDouble(controller_frequency_param_name,20.0);

                if(controller_frequency > 0){
                    sim_period_ = 1.0 / controller_frequency;
                }else{
                    this.node.getLog().warn("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
                    sim_period_ = 0.05;
                }
            }
            this.node.getLog().info("Sim period is set to "+ String.format("%1$.2f",sim_period_));

            me.simPeriod=sim_period_;

            //other params agent
            //me.time_horizon_obst_ = getParamDef(private_nh,"time_horizon_obst",10.0); currently not used in agent
            me.time_to_holo_ = params.getDouble("time_to_holo", 0.4);
            me.minErrorHolo = params.getDouble("min_error_holo", 0.01);
            me.maxErrorHolo = params.getDouble( "max_error_holo", 0.15);
            //delete_observations_ = params.getBoolean("delete_observations", true); currently not used in agent
            //threshold_last_seen_ = params.getDouble("threshold_last_seen",1.0); currently not used in agent
            me.eps_= params.getDouble( "eps", 0.1);

            boolean orca, convex, clearpath, use_truncation;
            int num_samples, type_vo;
            me.orca=params.getBoolean( "orca");
            me.convex=params.getBoolean( "convex");
            //params.getBoolean( "clearpath", &clearpath); not used
            me.useTruancation=params.getBoolean( "use_truncation");

            //num_samples = getParamDef(private_nh, "num_samples", 400); not used
            me.voType = params.getInteger("type_vo", 0); //HRVO

            me.truncTime = params.getDouble("trunc_time",5.0);
            //left_pref_ = getParamDef(private_nh,"left_pref",0.1); not used

            me.publishPositionsPeriod = 1.0/params.getDouble("publish_positions_frequency",10.0);

            me.publishMePeriod = 1.0/params.getDouble("publish_me_frequency",10.0);

            //set Footprint
            List<Point> footprint_points=new ArrayList<Point>();
            //TODO:footprint_points = costmap_ros_->getRobotFootprint();

            PolygonStamped footprint=messageFactory.newFromType(PolygonStamped._TYPE);
            Point32 p=messageFactory.newFromType(Point32._TYPE);
            List<Point32> points=new ArrayList<Point32>();
            for (int i = 0; i<footprint_points.size(); i++) {
                p.setX((float)footprint_points.get(i).getX());
                p.setY((float) footprint_points.get(i).getY());
                points.add(p);
            }
            Polygon polygon=messageFactory.newFromType(Polygon._TYPE);
            polygon.setPoints(points);
            footprint.setPolygon(polygon);

            points.clear();
            if (footprint.getPolygon().getPoints().size()>2)
                me.setFootprint(footprint);
            else {
                double angle = 0;
                double step = 2 * Math.PI / 72;
                while(angle < 2 * Math.PI){
                    Point32 pt=messageFactory.newFromType(Point32._TYPE);
                    pt.setX((float)(radius_ * Math.cos(angle)));
                    pt.setY((float) (radius_ * Math.sin(angle)));
                    pt.setZ(0.0f);
                    points.add(pt);
                    angle += step;
                }
                polygon.setPoints(points);
                footprint.setPolygon(polygon);
                me.setFootprint(footprint);
            }

            me.initAsMe(this.node, tf_);
            me.id_=new String(my_id);



            skip_next_ = false;
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

            setup_= false;
            //not implemented yet
            //dsrv_ = new dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>(private_nh);
            //dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>::CallbackType cb = boost::bind(&CollvoidLocalPlanner::reconfigureCB, this, _1, _2);
            //dsrv_->setCallback(cb);

            initialized_ = true;
        }
        else {
            node.getLog().info("This planner has already been initialized, you can't call it twice, doing nothing");
        }

    }
    /*Agent me init done*/

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

    //the interface isGoalReached
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
        Vector3 linear=messageFactory.newFromType(Vector3._TYPE);
        Vector3 angular=messageFactory.newFromType(Vector3._TYPE);


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
            base_local_planner::publishPlan(transformed_plan_, g_plan_pub_);
            base_local_planner::publishPlan(transformed_plan_, l_plan_pub_);
            //we don't actually want to run the controller when we're just rotating to goal
            return true;
        }

        tf::Stamped<tf::Pose> target_pose;
        target_pose.setIdentity();
        target_pose.frame_id_ = robot_base_frame_;

        if (!skip_next_){
            if(!transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, global_frame_, transformed_plan_)){
                ROS_WARN("Could not transform the global plan to the frame of the controller");
                return false;
            }
            geometry_msgs::PoseStamped target_pose_msg;
            findBestWaypoint(target_pose_msg, global_pose);
        }
        tf::poseStampedMsgToTF(transformed_plan_[current_waypoint_], target_pose);


        geometry_msgs::Twist res;

        res.linear.x = target_pose.getOrigin().x() - global_pose.getOrigin().x();
        res.linear.y = target_pose.getOrigin().y() - global_pose.getOrigin().y();
        res.angular.z = angles::shortest_angular_distance(tf::getYaw(global_pose.getRotation()),atan2(res.linear.y, res.linear.x));


        collvoid::Vector2 goal_dir = collvoid::Vector2(res.linear.x,res.linear.y);
        // collvoid::Vector2 goal_dir = collvoid::Vector2(goal_x,goal_y);
        if (collvoid::abs(goal_dir) > max_vel_x_) {
            goal_dir = max_vel_x_ * collvoid::normalize(goal_dir);
        }
        else if (collvoid::abs(goal_dir) < min_vel_x_) {
            goal_dir = min_vel_x_ * 1.2* collvoid::normalize(goal_dir);
        }


        collvoid::Vector2 pref_vel = collvoid::Vector2(goal_dir.x(),goal_dir.y());

        //TODO collvoid added

        me_->computeNewVelocity(pref_vel, cmd_vel);


        if(std::abs(cmd_vel.angular.z)<min_vel_th_)
        cmd_vel.angular.z = 0.0;
        if(std::abs(cmd_vel.linear.x)<min_vel_x_)
        cmd_vel.linear.x = 0.0;
        if(std::abs(cmd_vel.linear.y)<min_vel_y_)
        cmd_vel.linear.y = 0.0;

        bool valid_cmd = true; //collision_planner_.checkTrajectory(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,true);

        if (!valid_cmd){
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
        }

        if (cmd_vel.linear.x == 0.0 && cmd_vel.angular.z == 0.0 && cmd_vel.linear.y == 0.0) {

            ROS_DEBUG("Did not find a good vel, calculated best holonomic velocity was: %f, %f, cur wp %d of %d trying next waypoint", me_->velocity_.x(),me_->velocity_.y(), current_waypoint_, (int)transformed_plan_.size());
            if (current_waypoint_ < transformed_plan_.size()-1){
                current_waypoint_++;
                skip_next_= true;
            }
            else {
                transformed_plan_.clear();
                base_local_planner::publishPlan(transformed_plan_, g_plan_pub_);
                base_local_planner::publishPlan(transformed_plan_, l_plan_pub_);

                return false;
            }
        }
        else {
            skip_next_ = false;
        }


        std::vector<geometry_msgs::PoseStamped> local_plan;
        geometry_msgs::PoseStamped pos;
        //pos.header.frame_id = robot_base_frame_;

        tf::poseStampedTFToMsg(global_pose,pos);
        local_plan.push_back(pos);
        local_plan.push_back(transformed_plan_[current_waypoint_]);
        base_local_planner::publishPlan(transformed_plan_, g_plan_pub_);
        base_local_planner::publishPlan(local_plan, l_plan_pub_);
        //me_->publishOrcaLines();
        return true;
    }

}
