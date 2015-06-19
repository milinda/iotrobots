package cgl.iotrobots.collavoid.commons.planners;

import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.PoseStamped_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;

import java.util.ArrayList;
import java.util.List;

public class Agent extends Neighbor {

    //parameters read directly from configuration, or set automatically
    public double rot_stopped_velocity;
    public double trans_stopped_velocity;
    public double yaw_goal_tolerance;
    public double xy_goal_tolerance;
    public boolean latch_xy_goal_tolerance;
    public boolean ignore_goal_yaw;
    public boolean use_obstacles_;
    public String global_frame_;
    public double acc_lim_x_;
    public double acc_lim_y_;
    public double acc_lim_th_;
    public double max_vel_with_obstacles_;
    public double min_vel_x_;
    public double max_vel_x_;
    public double min_vel_y_;
    public double max_vel_y_;
    public double max_vel_th_;
    public double min_vel_th_;
    public double min_vel_th_inplace_;
    public double footprint_radius_;
    public double minErrorHolo;
    public double maxErrorHolo;
    public double time_to_holo_;
    public double eps_;
    public boolean orca;
    public boolean convex;
    public boolean useTruancation;
    public int voType;
    public double truncTime;
    public double publishMePeriod;

    //paramters calculated from configuration
    public String base_frame_;
    public double wheel_base_;
    public List<Vector2> footprint_original;

    //robot state information
    public Vector2 prefVelociy;
    public Vector2 newVelocity;
    public Twist_ cmd_vel;
    public double cur_allowed_nh_error_;
    public double max_speed_x_; //in nonholomic robot, it has only one liner velocity
    public double min_dist;
    public boolean has_polygon_footprint_;
    public double cur_loc_unc_radius_;
    public List<Line> addOrcaLines;
    public List<VO> voAgents;
    public List<Neighbor> AgentNeighbors;
    public List<Obstacle> obstacles_from_laser_;
    public List<LinePair> footprint_minkowski_lines;
    public List<PoseStamped_> plan;
    public List<PoseStamped_> transformed_plan;
    public boolean isNew = true;//not used
    public int current_waypoint = 0;
    public boolean skip_next = false;
    public boolean xy_tolerance_latch_ = false;
    public boolean rotating_to_goal_ = false;

    // inherit from Neighbor
//    public String Name;
//    public boolean holo_robot_;
//    public Odometry_ base_odom_;
//    public double radius;
//    public boolean controlled;
//    private List<Vector2> footprint_minkowski;// with localization uncertainty
//    public List<Vector2> footPrint_rotated;
//    public long last_seen_;
//    public Position position;
//    public Vector2 velocity;
//    public double controlPeriod;

    //-----------------------method begin-------------------------------//

    public Agent() {
        super();
    }

    public Agent(String id, String name) {
        super(id, name);
        initParameters();
    }

    public Agent(String id) {
        super(id);
        initParameters();
    }

    void initParameters() {
        cur_allowed_nh_error_ = 0;
        cur_loc_unc_radius_ = 0;// not used
        base_odom_ = null;
        cmd_vel = new Twist_();
        prefVelociy = null;
        newVelocity = null;
        plan = new ArrayList<PoseStamped_>();
        transformed_plan = new ArrayList<PoseStamped_>();
        footprint_original = new ArrayList<Vector2>();
        footprint_minkowski = new ArrayList<Vector2>();
        footprint_minkowski_lines = new ArrayList<LinePair>();
        AgentNeighbors = new ArrayList<Neighbor>();
        obstacles_from_laser_ = new ArrayList<Obstacle>();
        addOrcaLines = new ArrayList<Line>();
        voAgents = new ArrayList<VO>();

        double controller_frequency = -1;

        //load parameters locally
        rot_stopped_velocity = Parameters.ROT_STOPPED_VELOCITY;
        trans_stopped_velocity = Parameters.TRANS_STOPPED_VELOCITY;
        yaw_goal_tolerance = Parameters.YAW_GOAL_TOLERANCE;
        xy_goal_tolerance = Parameters.XY_GOAL_TOLERANCE;
        latch_xy_goal_tolerance = Parameters.LATCH_XY_GOAL_TOLERANCE;
        ignore_goal_yaw = Parameters.IGNORE_GOAL_YAW;

        use_obstacles_ = Parameters.USE_OBSTACLES;
        controlled = Parameters.CONTROLLED;

        updateBaseFrame();//get rid of the / character
        global_frame_ = Parameters.GLOBAL_FRAME;

        //acceleration limits, from params_turtle.yaml
        acc_lim_x_ = Parameters.ACC_LIM_X;
        acc_lim_y_ = Parameters.ACC_LIM_Y;
        acc_lim_th_ = Parameters.ACC_LIM_TH;

        //holo_robot
        holo_robot_ = Parameters.HOLO_ROBOT;
        if (!holo_robot_)
            wheel_base_ = Parameters.WHEEL_BASE;
        else
            wheel_base_ = 0.0;

        //min max speeds
        max_vel_with_obstacles_ = Parameters.MAX_VEL_WITH_OBSTACLES;
        max_vel_x_ = Parameters.MAX_VEL_X;
        min_vel_x_ = Parameters.MIN_VEL_X;
        max_vel_y_ = Parameters.MAX_VEL_Y;
        min_vel_y_ = Parameters.MIN_VEL_Y;
        max_vel_th_ = Parameters.MAX_VEL_TH;
        min_vel_th_ = Parameters.MIN_VEL_TH;
        min_vel_th_inplace_ = Parameters.MIN_VEL_TH_INPLACE;

        max_speed_x_ = max_vel_x_;

        //set radius
        footprint_radius_ = Parameters.FOOTPRINT_RADIUS;

        // set radius with localization error, as we use footprint to calculate, this is set to footprint radius
        radius = footprint_radius_ + cur_loc_unc_radius_;
        //sim period
        controller_frequency = Parameters.CONTROLLER_FREQUENCY;

        //me.time_horizon_obst_ = getParamDef(private_nh,"time_horizon_obst",10.0); currently not used in agent

        //non holo robot parameters
        time_to_holo_ = Parameters.TIME_TO_HOLO;
        minErrorHolo = Parameters.MIN_ERROR_HOLO;
        maxErrorHolo = Parameters.MAX_ERROR_HOLO;

        //delete_observations_ = params.getBoolean("delete_observations", true); currently not used in agent
        //threshold_last_seen_ = params.getDouble("threshold_last_seen",1.0); currently not used in agent

        //parameters for convex footprint which considers localization uncertainty
        eps_ = Parameters.EPS;
        orca = Parameters.ORCA;
        convex = Parameters.CONVEX;
        //params.getBoolean( "clearpath", &clearpath); not used as we only use clear path method
        useTruancation = Parameters.USE_TRUNCATION;

        //num_samples = getParamDef(private_nh, "num_samples", 400); not used
        voType = Parameters.TYPE_VO; //HRVO      0:HRVO,1:RVO,2:VO

        truncTime = Parameters.TRUNC_TIME;
        //left_pref_ = getParamDef(private_nh,"left_pref",0.1); not used as it is for orca method

        //position share publish frequency
        publishMePeriod = 1.0 / Parameters.PUBLISH_ME_FREQUENCY;

        if (controller_frequency <= 0) {
            System.err.println("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
            controlPeriod = 0.05;
        } else {
            controlPeriod = 1.0 / controller_frequency;
        }

        //currently use circular footprint
        footprint_original.clear();
        double angle = 0;
        double step = 2 * Math.PI / 72;
        while (angle < 2 * Math.PI) {
            Vector2 pt = new Vector2();
            pt.setX((float) (footprint_radius_ * Math.cos(angle)));
            pt.setY((float) (footprint_radius_ * Math.sin(angle)));
            footprint_original.add(pt);
            angle += step;
        }
        setFootprint_minkowski(footprint_original);
    }

    public void updateBaseFrame() {
        base_frame_ = Name.replace("/", "") + Parameters.BASE_FRAME_SUFFIX;//get rid of the / character
    }

    /*++++++++++++++++++++++++Get stuff++++++++++++++++++++++++++*/
    public double getRadius() {
        return radius;
    }

    public Odometry_ getBaseOdom() {
        return base_odom_;
    }

    public long getLastSeen() {
        return last_seen_;
    }

    public String getName() {
        return Name;
    }

    public double getPublishMePeriod() {
        return publishMePeriod;
    }

    public long getLastTimeMePublished() {
        return lastTimeMePublished;
    }

    public String getBase_frame_() {
        return base_frame_;
    }

    public String getGlobal_frame_() {
        return global_frame_;
    }

    public boolean getHoloRobot() {
        return holo_robot_;
    }

    public boolean getControlled() {
        return controlled;
    }

    public double getCur_loc_unc_radius_() {
        return cur_loc_unc_radius_;
    }

    public List<Vector2> getFootprint_minkowski() {
        return footprint_minkowski;
    }

    public double getFootprint_radius_() {
        return footprint_radius_;
    }

    public Position getPosition() {
        return position;
    }

    public List<Neighbor> getAgentNeighbors() {
        return AgentNeighbors;
    }

    public List<Obstacle> getObstacles_from_laser_() {
        return obstacles_from_laser_;
    }

    public List<Vector2> getFootprint_original() {
        return footprint_original;
    }

    public List<PoseStamped_> getPlan() {
        return plan;
    }

    public boolean isNew() {
        return isNew;
    }
/*+++++++++++++++++++++++++Get stuff end+++++++++++++++++++++++++*/

    /*++++++++++++++++++++++++Set stuff++++++++++++++++++++++++++*/

    public void setNew(boolean isNew) {
        this.isNew = isNew;
    }

    public void setPlan(List<PoseStamped_> plan) {
        this.plan = plan;
    }

    public void setHolo_robot_(boolean holo_robot_) {
        this.holo_robot_ = holo_robot_;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public void setControlled(boolean controlled) {
        this.controlled = controlled;
    }

    public void setVelocity(Vector2 velocity) {
        this.velocity = velocity;
    }

    public void setFootPrint_rotated(List<Vector2> footPrint_rotated) {
        this.footprint_rotated = footPrint_rotated;
    }

    public void setPosition(Position position) {
        this.position = position;
    }

    public void setLast_seen_(long last_seen_) {
        this.last_seen_ = last_seen_;
    }

    public void setLastTimeMePublished(long lastTimeMePublished) {
        this.lastTimeMePublished = lastTimeMePublished;
    }

    public void setPrefVelociy(Vector2 prefVelociy) {
        this.prefVelociy = prefVelociy;
    }

    public void setCur_allowed_nh_error_(double cur_allowed_nh_error_) {
        this.cur_allowed_nh_error_ = cur_allowed_nh_error_;
    }

    public void setMax_speed_x_(double max_speed_x_) {
        this.max_speed_x_ = max_speed_x_;
    }

    public void setMin_dist(double min_dist) {
        this.min_dist = min_dist;
    }

    public void setHas_polygon_footprint_(boolean has_polygon_footprint_) {
        this.has_polygon_footprint_ = has_polygon_footprint_;
    }

    public void setCur_loc_unc_radius_(double cur_loc_unc_radius_) {
        this.cur_loc_unc_radius_ = cur_loc_unc_radius_;
    }

    public void setAgentNeighbors(List<Neighbor> agentNeighbors) {
        AgentNeighbors = agentNeighbors;
    }

    public void setObstacles_from_laser_(List<Obstacle> obstacles_from_laser_) {
        this.obstacles_from_laser_ = obstacles_from_laser_;
    }

    // footprint(minkowski footprint) are all in robot frame
    // set both footprint minkowski and the footprint minkowski lines
    public void setFootprint_minkowski(List<Vector2> footprint) {
        if (footprint.size() < 2) {
            throw new RuntimeException("The footprint specified has less than two nodes");
        }
        this.lock.lock();
        try {
            footprint_minkowski.clear();
            for (Vector2 pt : footprint) {
                footprint_minkowski.add(new Vector2(pt.getX(), pt.getY()));
            }
            footprint_minkowski_lines.clear();
            Vector2 p = footprint.get(0);
            Vector2 first = new Vector2(p.getX(), p.getY());
            Vector2 old = new Vector2(p.getX(), p.getY());
            //add linesegments for footprint
            for (int i = 0; i < footprint.size(); i++) {
                footprint_minkowski_lines.add(new LinePair(old, footprint.get(i)));
                old.setVector2(footprint.get(i));
            }
            //add last segment
            footprint_minkowski_lines.add(new LinePair(old, first));
            has_polygon_footprint_ = true;
        } finally {
            this.lock.unlock();
        }
    }

    @Override
    public void setSeq(long seq) {
        super.setSeq(seq);
        cmd_vel.setSeq(seq);
    }

    /*+++++++++++++++++++++++++Set stuff end+++++++++++++++++++++++++*/
}

