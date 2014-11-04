package cgl.iotrobots.collavoid.LocalPlanner;

import cgl.iotrobots.collavoid.ROSAgent.Agent;
import geometry_msgs.*;
import nav_msgs.Path;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.pubsub.TransformListener;
import costmap_2d.VoxelGrid;

import java.net.InetAddress;
import java.net.UnknownHostException;
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

    Agent me_;

    double time_to_holo_, min_error_holo_, max_error_holo_;

    String global_frame_; ///< @brief The frame in which the controller will run
    String robot_base_frame_; ///< @brief Used as the base frame id of the robot
    List<PoseStamped> global_plan_, transformed_plan_;
    Publisher g_plan_pub_, l_plan_pub_;
    Subscriber obstacles_sub_;

    private static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    private static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);


    /*---------------methods begin-----------------*/

    //must pass connectednode
    public LocalPlanner(final ConnectedNode node, TransformListener tf, VoxelGrid costmap_ros){
        costmap_ros_=null;
        tf_=null;
        initialized_=false;
        this.node=node;
        //initialize the planner
        initialize(tf, costmap_ros);
    }

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
            Agent me=new Agent();

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
            List<Point> footprint_points;
            //TODO:footprint_points = costmap_ros_->getRobotFootprint();

            PolygonStamped footprint=messageFactory.newFromType(PolygonStamped._TYPE);
            Point32 p=messageFactory.newFromType(Point32._TYPE);
            List<Point32> points;
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
                me_.setFootprint(footprint);
            else {
                double angle = 0;
                double step = 2 * Math.PI / 72;
                while(angle < 2 * Math.PI){
                    Point32 pt=messageFactory.newFromType(Point32._TYPE);
                    pt.setX((float)(radius_ * Math.cos(angle)));
                    pt.setY((float) (radius_ * Math.sin(angle)));
                    pt.setZ(0.0f);
                    points.add(pt);
                    footprint.polygon.points.push_back(pt);
                    angle += step;
                }
                polygon.setPoints(points);
                footprint.setPolygon(polygon);
                me_.setFootprint(footprint);
            }

            me_.initAsMe(this.node, tf_);
            me_.id_=new String(my_id);



            skip_next_ = false;
            Publisher<Path> g_plan_pub_=this.node.newPublisher("global_plan", Path._TYPE);
            Publisher<Path> l_plan_pub_=this.node.newPublisher("l_plan_pub_", Path._TYPE);
            String move_base_name = this.node.getName().toString();
            //ROS_ERROR("%s name of node", thisname.c_str());

            Subscriber<std_msgs.String> obstacles_sub_ = this.node.newSubscriber(move_base_name + "/local_costmap/obstacles", std_msgs.String._TYPE);
            obstacles_sub_.addMessageListener(new MessageListener<std_msgs.String>() {
                @Override
                public void onNewMessage(std_msgs.String message) {
                    log.info("I heard: \"" + message.getData() + "\"");
                }
            });
            obstacles_sub_ = nh.subscribe(move_base_name + "/local_costmap/obstacles",1,&CollvoidLocalPlanner::obstaclesCallback,this);

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
        //end if init
    } // end init

}
