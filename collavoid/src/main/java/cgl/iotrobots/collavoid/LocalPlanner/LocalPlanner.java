package cgl.iotrobots.collavoid.LocalPlanner;

import cgl.iotrobots.collavoid.ROSAgent.Agent;
import geometry_msgs.PoseStamped;
import org.ros.node.AbstractNodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.pubsub.TransformListener;
import costmap_2d.VoxelGrid;
import java.util.List;

/**
 * Created by hjh on 11/3/14.
 */

public class LocalPlanner extends AbstractNodeMain {
    //currently no dynamic reconfigurer

    //Datatypes:
    VoxelGrid costmap_ros_;
    VoxelGrid costmap_; ///< @brief The costmap the controller will use

    TransformListener tf_;

    boolean initialized_, skip_next_, setup_;

    //Agent stuff
    double sim_period_;
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

    Agent me_;

    double time_to_holo_, min_error_holo_, max_error_holo_;

    String global_frame_; ///< @brief The frame in which the controller will run
    String robot_base_frame_; ///< @brief Used as the base frame id of the robot
    List<PoseStamped> global_plan_, transformed_plan_;
    Publisher g_plan_pub_, l_plan_pub_;
    Subscriber obstacles_sub_;

    /*---------------methods begin-----------------*/


    public void LocalPlanner(){
        costmap_ros_=null;
        tf_=null;
        initialized_=false;
    }

    public void LocalPlanner(String name, TransformListener tf, VoxelGrid costmap_ros){
        costmap_ros_=null;
        tf_=null;
        initialized_=false;
        //initialize the planner
        initialize(name, tf, costmap_ros);
    }

    private void initialize(String name, TransformListener tf, VoxelGrid costmap_ros){
        if (!initialized_){
            tf_ = tf;
            costmap_ros_ = costmap_ros;

            rot_stopped_velocity_ = 1e-2;
            trans_stopped_velocity_ = 1e-2;

            current_waypoint_ = 0;

            ros::NodeHandle private_nh("~/" + name);


            //base local planner params
            yaw_goal_tolerance_ = getParamDef(private_nh,"yaw_goal_tolerance", 0.05);
            xy_goal_tolerance_  = getParamDef(private_nh,"xy_goal_tolerance", 0.10);
            latch_xy_goal_tolerance_ = getParamDef(private_nh,"latch_xy_goal_tolerance", false);
            ignore_goal_yaw_ = getParamDef(private_nh, "ignore_goal_jaw", false);


            ros::NodeHandle nh;//for stand alone??


            //set my id
            String my_id = nh.getNamespace();
            if (strcmp(my_id.c_str(), "/") == 0) {
                char hostname[1024];
                hostname[1023] = '\0';
                gethostname(hostname,1023);
                my_id = String(hostname);
            }
            my_id = getParamDef<String>(private_nh,"name",my_id);
            ROS_INFO("My name is: %s",my_id.c_str());


            //init ros agent
            me_.reset(new ROSAgent());

            //acceleration limits load from params_turtle.yaml and private_nh
            //namespace maybe /CollvoidLocalPlanner
            getParam(private_nh,"acc_lim_x", &acc_lim_x_ );
            getParam(private_nh,"acc_lim_y", &acc_lim_y_ );
            getParam(private_nh,"acc_lim_th", &acc_lim_th_ );

            me_->setAccelerationConstraints(acc_lim_x_, acc_lim_y_, acc_lim_th_);

            //holo_robot
            getParam(private_nh, "holo_robot",&holo_robot_);
            me_->setIsHoloRobot(holo_robot_);

            if (!holo_robot_)
                getParam(private_nh,"wheel_base", &wheel_base_);
            else
            wheel_base_ = 0.0;
            me_->setWheelBase(wheel_base_);


            //min max speeds
            getParam(private_nh,"max_vel_with_obstacles", &max_vel_with_obstacles_);
            me_->setMaxVelWithObstacles(max_vel_with_obstacles_);

            getParam(private_nh,"max_vel_x", &max_vel_x_);
            getParam(private_nh,"min_vel_x", &min_vel_x_);
            getParam(private_nh,"max_vel_y", &max_vel_y_);
            getParam(private_nh,"min_vel_y", &min_vel_y_);
            getParam(private_nh,"max_vel_th", &max_vel_th_);
            getParam(private_nh,"min_vel_th", &min_vel_th_);
            getParam(private_nh,"min_vel_th_inplace", &min_vel_th_inplace_);

            me_->setMinMaxSpeeds(min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, min_vel_th_, max_vel_th_, min_vel_th_inplace_);

            //set radius
            getParam(private_nh,"footprint_radius",&radius_);
            me_->setFootprintRadius(radius_);

            //set frames
            robot_base_frame_ = costmap_ros_->getBaseFrameID();
            global_frame_ = getParamDef<String>(private_nh,"global_frame", "/map");
            me_->setGlobalFrame(global_frame_);
            me_->setRobotBaseFrame(robot_base_frame_);

            //sim period
            String controller_frequency_param_name;
            if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
                sim_period_ = 0.05;
            else
            {
                double controller_frequency = 0;
                private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
                if(controller_frequency > 0)
                    sim_period_ = 1.0 / controller_frequency;
                else
                {
                    ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
                    sim_period_ = 0.05;
                }
            }
            ROS_INFO("Sim period is set to %.2f", sim_period_);
            me_->setSimPeriod(sim_period_);



            //other params agent
            time_horizon_obst_ = getParamDef(private_nh,"time_horizon_obst",10.0);
            time_to_holo_ = getParamDef(private_nh,"time_to_holo", 0.4);
            min_error_holo_ = getParamDef(private_nh,"min_error_holo", 0.01);
            max_error_holo_ = getParamDef(private_nh, "max_error_holo", 0.15);
            delete_observations_ = getParamDef(private_nh, "delete_observations", true);
            threshold_last_seen_ = getParamDef(private_nh,"threshold_last_seen",1.0);
            eps_= getParamDef(private_nh, "eps", 0.1);

            bool orca, convex, clearpath, use_truncation;
            int num_samples, type_vo;
            getParam(private_nh, "orca", &orca);
            getParam(private_nh, "convex", &convex);
            getParam(private_nh, "clearpath", &clearpath);
            getParam(private_nh, "use_truncation", &use_truncation);

            num_samples = getParamDef(private_nh, "num_samples", 400);
            type_vo = getParamDef(private_nh, "type_vo", 0); //HRVO


            me_->setTimeHorizonObst(time_horizon_obst_);
            me_->setTimeToHolo(time_to_holo_);
            me_->setMinMaxErrorHolo(min_error_holo_, max_error_holo_);
            me_->setDeleteObservations(delete_observations_);
            me_->setThresholdLastSeen(threshold_last_seen_);
            me_->setLocalizationEps(eps_);

            me_->setOrca(orca);
            me_->setClearpath(clearpath);
            me_->setTypeVO(type_vo);
            me_->setConvex(convex);
            me_->setUseTruncation(use_truncation);
            me_->setNumSamples(num_samples);


            trunc_time_ = getParamDef(private_nh,"trunc_time",5.0);
            left_pref_ = getParamDef(private_nh,"left_pref",0.1);

            publish_positions_period_ = getParamDef(private_nh,"publish_positions_frequency",10.0);
            publish_positions_period_ = 1.0 / publish_positions_period_;

            publish_me_period_ = getParamDef(private_nh,"publish_me_frequency",10.0);
            publish_me_period_ = 1.0/publish_me_period_;

            me_->setTruncTime(trunc_time_);
            me_->setLeftPref(left_pref_);
            me_->setPublishPositionsPeriod(publish_positions_period_);
            me_->setPublishMePeriod(publish_me_period_);




            //set Footprint
            std::vector<geometry_msgs::Point> footprint_points;
            footprint_points = costmap_ros_->getRobotFootprint();

            geometry_msgs::PolygonStamped footprint;
            geometry_msgs::Point32 p;
            for (int i = 0; i<(int)footprint_points.size(); i++) {
                p.x = footprint_points[i].x;
                p.y = footprint_points[i].y;
                footprint.polygon.points.push_back(p);
            }
            if (footprint.polygon.points.size()>2)
                me_->setFootprint(footprint);
            else {
                double angle = 0;
                double step = 2 * M_PI / 72;
                while(angle < 2 * M_PI){
                    geometry_msgs::Point32 pt;
                    pt.x = radius_ * cos(angle);
                    pt.y = radius_ * sin(angle);
                    pt.z = 0.0;
                    footprint.polygon.points.push_back(pt);
                    angle += step;
                }
                me_->setFootprint(footprint);
            }

            me_->initAsMe(tf_);
            me_->setId(my_id);



            skip_next_ = false;
            g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
            String move_base_name = ros::this_node::getName();
            //ROS_ERROR("%s name of node", thisname.c_str());
            obstacles_sub_ = nh.subscribe(move_base_name + "/local_costmap/obstacles",1,&CollvoidLocalPlanner::obstaclesCallback,this);

            setup_= false;
            dsrv_ = new dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>(private_nh);
            dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>::CallbackType cb = boost::bind(&CollvoidLocalPlanner::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
        //end if init
    } // end init

}
