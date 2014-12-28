package cgl.iotrobots.collavoid.commons.planners;

import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Vector3d_;
import com.rabbitmq.client.Address;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Logger;

public class Agent {

    //config for simulation and some rules
    public boolean useTruancation;
    public double truncTime;
    public double ControlPeriod;

    //whether it is a controlled robot
    public boolean controlled;

    //setting for different CollAvoid strategies
    //orca or vo, need to figure out its usage
    public boolean orca;
    //footprint approximation, use circle approx or mink sum
    public boolean convex;

    //vo setting
    //clearPath or sampling based collAvoid strategy,use clearpath
    //boolean clearPath;

    //0:HRVO,1:RVO,2:VO
    public int voType;

    //robot information
    public Position position; //contains the heading information
    public Vector2 velocity;
    //    public Vector2 newVelocity;
    public Vector2 prefVelociy;
    public double radius;
    public List<Vector2> footPrint_rotated;

    //allowed error for non-holonomic robot
    public double cur_allowed_error_;

    //orca
    public double max_speed_x_; //in nonholomic robot it has only one liner velocity
    public List<Line> orcaLines, addOrcaLines;

    //VO
    public List<VO> voAgents;
//    public List<VelocitySample> samples;

    public List<Agent> AgentNeighbors;

    //config
    public double publishPositionsPeriod;
    public double publishMePeriod;//position share,in seconds

//    public double thresholdLastSeen;
//    public int numSamples;

    //helpers??
//    public long lastTimePositionsPublished;//for visualization in rviz
    public long lastTimeMePublished; //for position share

    //NH stuff
    public double minErrorHolo;
    public double maxErrorHolo;
    public boolean holo_robot_;
    public double time_to_holo_;

    //ORCA stuff
    public double max_vel_with_obstacles_;
    //public Vector2 holo_velocity_;

    //Obstacles
    public List<Obstacle> obstacles_from_laser_;

    public double min_dist;// for nh constraints

    //obstacles
    public boolean use_obstacles_;
    public List<Vector2> obstacle_points_;

    //Agent description
    public String Name;
    public String base_frame_;
    public String global_frame_;
    public double wheel_base_;
    public List<Vector2> footprint_original;// 2D footprint in robot frame
    public double acc_lim_x_, acc_lim_y_, acc_lim_th_;
    public double min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, max_vel_th_, min_vel_th_, min_vel_th_inplace_;
    public double footprint_radius_;

    //set automatically
    public boolean has_polygon_footprint_;

    public List<LinePair> footprint_original_lines;

    public long last_seen_;
    public Odometry_ base_odom_;

    //LOC uncertatiny
    public double eps_;
    public double cur_loc_unc_radius_;

    //COLLVOID
    public List<Vector2> footprint_minkowski;// with localization uncertainty
    public List<PoseWeighted> pose_array_weighted_;

    //lock
    public Lock me_lock_, obstacle_lock_, neighbors_lock_, convex_lock_;

    //utils
    private Logger logger;


    //-----------------------method begin-------------------------------//
    public Agent() {
        Name = "";
//        me_lock_ = new ReentrantLock();
//        obstacle_lock_ = new ReentrantLock();
//        neighbors_lock_ = new ReentrantLock();
//        convex_lock_ = new ReentrantLock();

        cur_allowed_error_ = 0;
        cur_loc_unc_radius_ = 0;// not used

        //holo_velocity_ = new Vector2();

        base_odom_ = new Odometry_();

        footprint_original = new ArrayList<Vector2>();
        footprint_minkowski = new ArrayList<Vector2>();
        footprint_original_lines = new ArrayList<LinePair>();
        obstacle_points_ = new ArrayList<Vector2>();
        pose_array_weighted_ = new ArrayList<PoseWeighted>();
        AgentNeighbors = new ArrayList<Agent>();
        obstacles_from_laser_ = new ArrayList<Obstacle>();
        addOrcaLines = new ArrayList<Line>();
        voAgents = new ArrayList<VO>();
//        samples = new ArrayList<VelocitySample>();

        logger = Logger.getLogger("Agent_Logger");
        initParameters();
    }

    // for neighbor recording
    public Agent(String name) {
        Name = name;
        footprint_original = new ArrayList<Vector2>();
        base_odom_ = new Odometry_();
        //holo_velocity_ = new Vector2();
        //set when update the agent state
        position = new Position();
        footprint_minkowski = new ArrayList<Vector2>();
    }

    void initParameters() {

        double controller_frequency = -1;

        //load parameters locally
        use_obstacles_ = Parameters.USE_OBSTACLES;
        controlled = Parameters.CONTROLLED;

        updateBaseFrame();//get rid of the / character
        global_frame_ = Parameters.GLOBAL_FRAME;

        //acceleration limits load from params_turtle.yaml
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

        //set radius
        footprint_radius_ = Parameters.FOOTPRINT_RADIUS;

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
        voType = Parameters.TYPE_VO; //HRVO

        truncTime = Parameters.TRUNC_TIME;
        //left_pref_ = getParamDef(private_nh,"left_pref",0.1); not used as it is for orca method

        //visualization publish frequency
        publishPositionsPeriod = 1.0 / Parameters.PUBLISH_POSITIONS_FREQUENCY;
        //position share publish frequency
        publishMePeriod = 1.0 / Parameters.PUBLISH_ME_FREQUENCY;

        radius = footprint_radius_ + cur_loc_unc_radius_;

        if (controller_frequency <= 0) {
            logger.warning("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
            ControlPeriod = 0.05;
        } else {
            ControlPeriod = 1.0 / controller_frequency;
        }
        logger.info("Sim period is set to " + String.format("%1$.2f", ControlPeriod));


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
        setFootprintOrignial(footprint_original);

//        logger.info("************************Agent " + Name + " is initialized!");
    }

    //called by local planner
    /* first refresh agent status according to the last velocity computed and then compute new
    * velocity */

    // cmd_vel is in robot frame that means it should have no Y velocity, just X and Z angular velocity
    public void updateState() {
//        me_lock_.lock();
//        try {
//            prefVelociy = pref_velocity;
            //Forward project agents,update me status like position and so on
            upDateAgentState(this);
            updateAllNeighbors();
//
//            newVelocity = new Vector2(0.0, 0.0);

//            addOrcaLines.clear();
//            voAgents.clear();

            computeMinDistToAll();

            max_speed_x_ = max_vel_x_;//???????????????????????????

            // currently only compute the clear path velocity which has the best performance as described in the thesis.

//            if (orca_) {
//                computeOrcaVelocity(pref_velocity);
//            } else {
//            samples.clear();
//                if (clearpath_) {
//            computeClearpathVelocity(pref_velocity);
//                } else {
//                    computeSampledVelocity(pref_velocity);
//                }
//            }
//
//
//            double speed_ang = Math.atan2(newVelocity.getY(), newVelocity.getX());
//            double dif_ang = Methods_Planners.shortest_angular_distance(position.getHeading(), speed_ang);
//
//            Vector3d_ linear = new Vector3d_();
//            Vector3d_ angular = new Vector3d_();
//            if (!holo_robot_) {
//                double vel = Vector2.abs(newVelocity);
//                double vstar;
//
//                if (Math.abs(dif_ang) > Parameters.EPSILON)
//                    vstar = Methods_Planners.NHORCA.calcVstar(vel, dif_ang);//get nonholomonic velocity
//                else
//                    vstar = max_vel_x_;
//
//                linear.setX(Math.min(vstar, vMaxAng()));
//                linear.setY(0.0);
//                cmd_vel.setLinear(linear);
//
//                //ROS_ERROR("dif_ang %f", dif_ang);
//                if (Math.abs(dif_ang) > 3.0 * Math.PI / 4.0) {
//                    angular.setZ(
//                            Methods_Planners.sign(
//                                    base_odom_.getTwist().getAngular().getZ()) *
//                                    Math.min(Math.abs(dif_ang / time_to_holo_),
//                                            max_vel_th_)
//                    );
//                    cmd_vel.setAngular(angular);
//                } else {
//                    angular.setZ(
//                            Methods_Planners.sign(dif_ang) *
//                                    Math.min(Math.abs(dif_ang / time_to_holo_),
//                                            max_vel_th_)
//                    );
//                    cmd_vel.setAngular(angular);
//                }
//                //ROS_ERROR("vstar = %.3f", vstar);
//            } else {
//                //new velocity is caculated in world frame, it has to be transformed to robot base frame
//                Vector2 rotated_vel = Vector2.rotateVectorByAngle(newVelocity, -position.getHeading());
//
//                linear.setX(rotated_vel.getX());
//                linear.setY(rotated_vel.getY());
//                cmd_vel.setLinear(linear);
//                if (min_dist > 2 * footprint_radius_) {
//                    angular.setZ(Methods_Planners.sign(dif_ang) * Math.min(Math.abs(dif_ang), max_vel_th_));
//                    cmd_vel.setAngular(angular);
//                }
//            }
//        } finally {
//            me_lock_.unlock();
//        }
    }

    //get closest ROSAgent/obstacle
    void computeMinDistToAll() {
        double min_dist_neigh = Double.MAX_VALUE;
        double min_dist_obstacle = Double.MAX_VALUE;
        //neighbors have already been sorted according to their dist to me
        if (AgentNeighbors.size() > 0)
            min_dist_neigh = Vector2.abs(Vector2.minus(AgentNeighbors.get(0).position.getPos(), position.getPos()));

        double threshold = Math.pow((Vector2.abs(velocity) + 4.0 * footprint_radius_), 2);
        for (Obstacle obstacle : obstacles_from_laser_) {
            double dist = Methods_Planners.distSqPointLineSegment(
                    obstacle.getBegin(),
                    obstacle.getEnd(),
                    position.getPos());
            obstacle.setDistToAgent(dist);
            if (dist < threshold) {
                if (dist < min_dist_obstacle) {
                    min_dist_obstacle = dist;
                }
            }

        }
        min_dist = Math.min(min_dist_neigh, min_dist_obstacle);

    }

    //update status of the agent according to its velocity
    void upDateAgentState(Agent agt) {
        double time_dif;
        if (agt.getLastSeen() == 0)
            time_dif = 0;
        else
            time_dif = System.currentTimeMillis() - agt.getLastSeen();
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
            y_dif = time_dif * agt.getBaseOdom().getTwist().getLinear().getY() * Math.sin(yaw + th_dif / 2.0);
        }
        theta = yaw + th_dif;
        x = agt.getBaseOdom().getPose().getPosition().getX() + x_dif;
        y = agt.getBaseOdom().getPose().getPosition().getY() + y_dif;
        agt.setPosition(new Position(x, y, theta));


        //minkowski footprint is in robot frame, in velocity space only need orientation.
        agt.setFootPrint_rotated(Methods_Planners.rotateFootprint(
                agt.getFootprint_minkowski(),
                agt.getPosition().getHeading()));

        //update velocity
        if (agt.getHoloRobot()) {
            x = agt.getBaseOdom().getTwist().getLinear().getX();
            y = agt.getBaseOdom().getTwist().getLinear().getY();
            pt.setX(x);
            pt.setY(y);
            agt.setVelocity(Vector2.rotateVectorByAngle(pt, (yaw + th_dif)));
        } else {//?????????????????????????????????????????????????????????????
            double dif_x, dif_y, dif_ang;
            dif_ang = agt.getControlPeriod() * agt.getBaseOdom().getTwist().getAngular().getZ();
            //in robot frame differential robot has only x velocity
            pt.setX(agt.getBaseOdom().getTwist().getLinear().getX() * Math.cos(dif_ang / 2.0));
            pt.setY(agt.getBaseOdom().getTwist().getLinear().getX() * Math.sin(dif_ang / 2.0));
            // in global frame, velocity need no translation
            agt.setVelocity(Vector2.rotateVectorByAngle(pt, (yaw + th_dif)));
        }
    }

    void updateAllNeighbors() {
        neighbors_lock_.lock();
        try {
            for (Agent neighbor : AgentNeighbors) {
                upDateAgentState(neighbor);
            }
            AgentNeighbors.sort(new Comparators.NeighborDistComparator(position.getPos()));
        } finally {
            neighbors_lock_.unlock();
        }
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

    public boolean getHoloRobot() {
        return holo_robot_;
    }

    public boolean getController() {
        return controlled;
    }

    //public Vector2 getHolo_velocity_() {
//        return holo_velocity_;
//    }

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

    public List<Agent> getAgentNeighbors() {
        return AgentNeighbors;
    }

    public Lock getObstacle_lock_() {
        return obstacle_lock_;
    }

    public Lock getConvex_lock_() {
        return convex_lock_;
    }

    public Lock getMe_lock_() {
        return me_lock_;
    }

    public Lock getNeighbors_lock_() {
        return neighbors_lock_;
    }

    public List<Obstacle> getObstacles_from_laser_() {
        return obstacles_from_laser_;
    }

    public List<Vector2> getFootprint_original() {
        return footprint_original;
    }
    
    /*+++++++++++++++++++++++++Get stuff end+++++++++++++++++++++++++*/

    /*++++++++++++++++++++++++Set stuff++++++++++++++++++++++++++*/

    public double getControlPeriod() {
        return ControlPeriod;
    }

    public void setHolo_robot_(boolean holo_robot_) {
        this.holo_robot_ = holo_robot_;
    }

//    public void setHolo_velocity_(Vector2 holo_velocity_) {
//        this.holo_velocity_ = holo_velocity_;
//    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public void setControlled(boolean controlled) {
        this.controlled = controlled;
    }

    public void setVelocity(Vector2 velocity) {
        this.velocity = velocity;
    }

    public void setFootprint_minkowski(List<Vector2> footprint_minkowski) {
        this.footprint_minkowski = footprint_minkowski;
    }

    public void setFootPrint_rotated(List<Vector2> footPrint_rotated) {
        this.footPrint_rotated = footPrint_rotated;
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

    // footprint(minkowski footprint) are all in robot frame
    public void setFootprintOrignial(List<Vector2> footprint) {
        if (footprint.size() < 2) {
            logger.severe("The footprint specified has less than two nodes");
            return;
        }

        footprint_minkowski.clear();
        for (Vector2 pt : footprint) {
            footprint_minkowski.add(pt);
        }

        footprint_original_lines.clear();
        Vector2 p = footprint.get(0);
        Vector2 first = new Vector2(p.getX(), p.getY());
        Vector2 old = new Vector2(p.getX(), p.getY());
        //add linesegments for footprint
        for (int i = 0; i < footprint.size(); i++) {
            footprint_original_lines.add(new LinePair(old, footprint.get(i)));
            old.setVector2(footprint.get(i));
        }
        //add last segment
        footprint_original_lines.add(new LinePair(old, first));
        has_polygon_footprint_ = true;
    }


    /*+++++++++++++++++++++++++Set stuff end+++++++++++++++++++++++++*/

}

