package cgl.iotrobots.collavoid.planners;

import cgl.iotrobots.collavoid.commons.planners.*;
import cgl.iotrobots.collavoid.commons.planners.Neighbor;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Vector3d_;
import com.rabbitmq.client.*;

import java.util.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Logger;

public class Agent extends Neighbor {

    //config for simulation and some rules
    public boolean useTruancation;
    public double truncTime;
    public double ControlPeriod;

    //whether it is a controlled robot
//    public boolean controlled;

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
//    public Position position; //contains the heading information
//    public Vector2 velocity;
    public Vector2 newVelocity;// calculated new holo velocity
//    private double radius;
//    public List<Vector2> footprint_rotated;

    //allowed error for non-holonomic robot
    public double cur_allowed_error_;

    //orca
    public double max_speed_x_; //in nonholomic robot it has only one liner velocity
    public List<Line> orcaLines, addOrcaLines;

    //VO
    public List<VO> voAgents;
    public List<VelocitySample> samples;

    public List<Neighbor> AgentNeighbors;

    //config
    public double publishPositionsPeriod;
    public double publishMePeriod;//position share,in seconds

    public double thresholdLastSeen;
    public int numSamples;

    //helpers??
    public long lastTimePositionsPublished;//for visualization in rviz
    public long lastTimeMePublished; //for position share

    //NH stuff
    public double minErrorHolo;
    public double maxErrorHolo;
    //    public boolean holo_robot_;
    public double time_to_holo_;

    //ORCA stuff
    public double max_vel_with_obstacles_;
    public Vector2 holo_velocity_;

    //Obstacles
    public List<Obstacle> obstacles_from_laser_;

    public double min_dist_obst_;

    //obstacles
    public boolean use_obstacles_;
    public List<Vector2> obstacle_points_;

    //Agent description
//    private String Name;
    public String base_frame_;
    public String global_frame_;
    public double wheel_base_;
    public List<Vector2> footprint_original;// 2D footprint in robot frame
    public double acc_lim_x_, acc_lim_y_, acc_lim_th_;
    public double min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, max_vel_th_, min_vel_th_, min_vel_th_inplace_;
    public double footprint_radius_;

    //set automatically
    public boolean has_polygon_footprint_;

    public List<LinePair> footprint_minkowski_lines;

//    private long last_seen_;
//    private Odometry_ base_odom_;

    //LOC uncertatiny
    public double eps_;
    public double cur_loc_unc_radius_;

    //COLLVOID
//    public List<Vector2> footprint_minkowski;
    public List<PoseWeighted> pose_array_weighted_;

    //lock
    public Lock me_lock_, obstacle_lock_, neighbors_lock_, convex_lock_;

    //utils
    private Logger logger;

    // message subscriber and publisher
    private RMQMsgManager rmqMsgManager;

    //-----------------------method begin-------------------------------//
    public Agent(String name,
                 Address[] addresses,
                 String url) {
        super(name);
        me_lock_ = new ReentrantLock();
        obstacle_lock_ = new ReentrantLock();
        neighbors_lock_ = new ReentrantLock();
        convex_lock_ = new ReentrantLock();

        cur_allowed_error_ = 0;
        cur_loc_unc_radius_ = 0;// not used
        min_dist_obst_ = Double.MAX_VALUE;

        holo_velocity_ = new Vector2();

        base_odom_ = new Odometry_();

        footprint_original = new ArrayList<Vector2>();
        footprint_minkowski = new ArrayList<Vector2>();
        footprint_minkowski_lines = new ArrayList<LinePair>();
        obstacle_points_ = new ArrayList<Vector2>();
        pose_array_weighted_ = new ArrayList<PoseWeighted>();
        AgentNeighbors = new ArrayList<Neighbor>();
        obstacles_from_laser_ = new ArrayList<Obstacle>();
        addOrcaLines = new ArrayList<Line>();
        voAgents = new ArrayList<VO>();
        samples = new ArrayList<VelocitySample>();
        rmqMsgManager = new RMQMsgManager(name, addresses, url);

        logger = Logger.getLogger(name + "Agent_Logger");
        initParameters();
    }

//    // for neighbor recording
//    public Agent(String name) {
//        Name = name;
//        footprint_original = new ArrayList<Vector2>();
//        base_odom_ = new Odometry_();
//        holo_velocity_ = new Vector2();
//        //set when update the agent state
//        position = new Position();
//        footprint_minkowski = new ArrayList<Vector2>();
//        footprint_minkowski_lines = new ArrayList<LinePair>();
//    }

    void initParameters() {

        double controller_frequency = -1;

        //load parameters locally
        use_obstacles_ = Parameters.USE_OBSTACLES;
        controlled = Parameters.CONTROLLED;

        base_frame_ = Name.replace("/", "") + Parameters.BASE_FRAME_SUFFIX;//get rid of the / character
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

        try {
            rmqMsgManager.start(this);
        } catch (Exception e) {
            e.printStackTrace();
        }

        logger.info("************************Agent " + Name + " is initialized!");
    }


    //called by local planner
    /* first refresh agent status according to the last velocity computed and then compute new
    * velocity */

    // cmd_vel is in robot frame that means it should have no Y velocity, just X and Z angular velocity
    public void computeNewVelocity(Vector2 pref_velocity, Twist_ cmd_vel) {
        me_lock_.lock();
        try {
            //Forward project agents,update me status like position and so on
            upDateAgentState(this);
            updateAllNeighbors();

            newVelocity = new Vector2(0.0, 0.0);

            addOrcaLines.clear();
            voAgents.clear();

            //get closest ROSAgent/obstacle
            double min_dist_neigh = Double.MAX_VALUE;
            if (AgentNeighbors.size() > 0)
                //neighbors have already been sorted according to their dist to me
                min_dist_neigh = Vector2.abs(Vector2.minus(AgentNeighbors.get(0).position.getPos(), position.getPos()));

//            System.out.println("min neighbor: "+min dist neigh);
//            System.out.println("obst_min: "+min dist obst_);
            double min_dist = Math.min(min_dist_neigh, min_dist_obst_);

            //incorporate NH constraints
            max_speed_x_ = max_vel_x_;//???????????????????????????

            if (!holo_robot_) {
                addNHConstraints(min_dist, pref_velocity);
            }
            //add acceleration constraints

            NHORCA.addAccelerationConstraintsXY(
                    max_vel_x_,
                    acc_lim_x_,
                    max_vel_y_,
                    acc_lim_y_,
                    velocity,
                    position.getHeading(),
                    ControlPeriod,
                    holo_robot_,
                    addOrcaLines
            );

            computeObstacles();

            // currently only compute the clear path velocity which has the best performance as described in the thesis.

//            if (orca_) {
//                computeOrcaVelocity(pref_velocity);
//            } else {
            samples.clear();
//                if (clearpath_) {
            computeClearpathVelocity(pref_velocity);
//                } else {
//                    computeSampledVelocity(pref_velocity);
//                }
//            }


            double speed_ang = Math.atan2(newVelocity.getY(), newVelocity.getX());
            double dif_ang = Methods_Planners.shortest_angular_distance(position.getHeading(), speed_ang);

            Vector3d_ linear = new Vector3d_();
            Vector3d_ angular = new Vector3d_();
            if (!holo_robot_) {
                double vel = Vector2.abs(newVelocity);
                double vstar;

                if (Math.abs(dif_ang) > Parameters.EPSILON)
                    vstar = NHORCA.calcVstar(vel, dif_ang);//get nonholomonic velocity
                else
                    vstar = max_vel_x_;

                linear.setX(Math.min(vstar, vMaxAng()));
                linear.setY(0.0);
                cmd_vel.setLinear(linear);

                //ROS_ERROR("dif_ang %f", dif_ang);
                if (Math.abs(dif_ang) > 3.0 * Math.PI / 4.0) {
                    angular.setZ(
                            Methods_Planners.sign(
                                    base_odom_.getTwist().getAngular().getZ()) *
                                    Math.min(Math.abs(dif_ang / time_to_holo_),
                                            max_vel_th_)
                    );
                    cmd_vel.setAngular(angular);
                } else {
                    angular.setZ(
                            Methods_Planners.sign(dif_ang) *
                                    Math.min(Math.abs(dif_ang / time_to_holo_),
                                            max_vel_th_)
                    );
                    cmd_vel.setAngular(angular);
                }
                //ROS_ERROR("vstar = %.3f", vstar);
            } else {
                //new velocity is caculated in world frame, it has to be transformed to robot base frame
                Vector2 rotated_vel = Vector2.rotateVectorByAngle(newVelocity, -position.getHeading());

                linear.setX(rotated_vel.getX());
                linear.setY(rotated_vel.getY());
                cmd_vel.setLinear(linear);
                if (min_dist > 2 * footprint_radius_) {
                    angular.setZ(Methods_Planners.sign(dif_ang) * Math.min(Math.abs(dif_ang), max_vel_th_));
                    cmd_vel.setAngular(angular);
                }
            }
        } finally {
            me_lock_.unlock();
        }
    }

    //update status of the agent according to its velocity
    void upDateAgentState(Neighbor agt) {
        agt.getLock().lock();
        try {
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
            agt.setFootprint_rotated(Methods_Planners.rotateFootprint(
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
        } finally {
            agt.getLock().unlock();
        }
    }

    void updateAllNeighbors() {
        neighbors_lock_.lock();
        try {
            for (int i = 0; i < AgentNeighbors.size(); i++) {
                upDateAgentState(AgentNeighbors.get(i));
            }
            AgentNeighbors.sort(new NeighborDistComparator(position.getPos().getX(), position.getPos().getY()));
        } finally {
            neighbors_lock_.unlock();
        }
    }


    void addNHConstraints(double min_dist, Vector2 pref_velocity) {
        double min_error = minErrorHolo;
        double max_error = maxErrorHolo;
        double error = max_error;
        double v_max_ang = vMaxAng();

        //ROS_ERROR("v_max_ang %.2f", v_max_ang);

        if (min_dist < 2.0 * footprint_radius_ + cur_loc_unc_radius_) {
            error = (max_error - min_error) / (Math.pow(2 * (footprint_radius_ + cur_loc_unc_radius_), 2)) * Math.pow(min_dist, 2) + min_error; // how much error do i allow?
            //ROS_DEBUG("Error = %f", error);
            if (min_dist < 0) {
                error = min_error;
                // ROS_DEBUG("%s I think I am in collision", me_->getId().c_str());
            }
        }
        cur_allowed_error_ = 1.0 / 3.0 * cur_allowed_error_ + 2.0 / 3.0 * error;
        //ROS_ERROR("error = %f", cur_allowed_nh_error_);
        double speed_ang = Math.atan2(pref_velocity.getY(), pref_velocity.getX());
        double dif_ang = Methods_Planners.shortest_angular_distance(position.getHeading(), speed_ang);
        //calculate possible tracking holomonic robot speed range
        if (Math.abs(dif_ang) > Math.PI / 2.0) { // || cur_allowed_nh_error_ < 2.0 * min_error) {
            double max_track_speed = NHORCA.calculateMaxTrackSpeedAngle(time_to_holo_, Math.PI / 2.0, cur_allowed_error_, max_vel_x_, max_vel_th_, v_max_ang);
            // tracking errors are not in velocity space!!!!!!!!!!!!!!
            if (max_track_speed <= 2 * min_error) {
                max_track_speed = 2 * min_error;
            }
            NHORCA.addMovementConstraintsDiffSimple(max_track_speed, position.getHeading(), addOrcaLines);
        } else {
            NHORCA.addMovementConstraintsDiff(cur_allowed_error_, time_to_holo_, max_vel_x_, max_vel_th_, position.getHeading(), v_max_ang, addOrcaLines);
        }
        max_speed_x_ = vMaxAng();

    }

    double vMaxAng() {
        //double theoretical_max_v = max_vel_th_ * wheel_base_ / 2.0;
        //return theoretical_max_v - std::abs(base_odom_.twist.twist.angular.z) * wheel_base_/2.0;
        return max_vel_x_; //TODO: fixme
    }


    void computeObstacles() {
        if (obstacles_from_laser_.size() <= 0)
            return;
        obstacle_lock_.lock();
        try {
            Vector<Vector2> own_footprint = new Vector<Vector2>();

            for (int i = 0; i < footprint_original.size(); i++) {
                own_footprint.add(new Vector2(
                                footprint_original.get(i).getX(),
                                footprint_original.get(i).getY())
                );
            }

            min_dist_obst_ = Double.MAX_VALUE;
            int i = 0;
            Vector<Integer> delete_list = new Vector<Integer>();
            for (Obstacle obst : obstacles_from_laser_) {
                if (!obst.getBegin().equals(obst.getEnd())) {
                    double dist = Methods_Planners.distSqPointLineSegment(obst.getBegin(), obst.getEnd(), position.getPos());
                    //why choose this limit?????????????????????????????????
                    if (dist < Math.pow((Vector2.abs(velocity) + 4.0 * footprint_radius_), 2)) {
                        if (use_obstacles_) {
                            if (orca) {// currently set to false
                                createObstacleLine(own_footprint, obst.getBegin(), obst.getEnd());
                            } else {//called from CP
                                VO obstacle_vo = ClearPathMethods.createObstacleVO(
                                        position.getPos(),
                                        footprint_radius_,
                                        footprint_rotated,
                                        obst.getBegin(),
                                        obst.getEnd()
                                );
                                voAgents.add(obstacle_vo);
                            }
                        }
                        if (dist < min_dist_obst_) {
                            min_dist_obst_ = dist;
                        }
                    }
                } else {
                    delete_list.add(i);
                }
                i++;
            }

            for (i = delete_list.size() - 1; i >= 0; i--) {
                obstacles_from_laser_.remove(delete_list.get(i));
            }

        } finally {
            obstacle_lock_.unlock();
        }
    }

    void createObstacleLine(Vector<Vector2> own_footprint, Vector2 obst1, Vector2 obst2) {

        double dist = Methods_Planners.distSqPointLineSegment(obst1, obst2, position.getPos());

        if (dist == Vector2.absSqr(Vector2.minus(position.getPos(), obst1))) {
            computeObstacleLine(obst1);
        } else if (dist == Vector2.absSqr(Vector2.minus(position.getPos(), obst2))) {
            computeObstacleLine(obst2);
        } else {
            Vector2 position_obst = Methods_Planners.projectPointOnLine(obst1, Vector2.minus(obst2, obst1), position.getPos());
            Vector2 rel_position = Vector2.minus(position_obst, position.getPos());
            dist = Math.sqrt(dist);
            double dist_to_footprint = getDistToFootprint(rel_position);
            if (dist_to_footprint == -1) {
                dist_to_footprint = footprint_radius_;
            }
            dist = dist - dist_to_footprint - 0.03;

            if (dist < 0.0) {
                Line line = new Line();
                line.setPoint(Vector2.scale(Vector2.normalize(rel_position), (dist - 0.02)));
                line.setDir(Vector2.normalize(Vector2.minus(obst1, obst2)));
                addOrcaLines.add(line);
                return;
            }

            if (Vector2.abs(Vector2.minus(position.getPos(), obst1)) > 2 * footprint_radius_ &&
                    Vector2.abs(Vector2.minus(position.getPos(), obst2)) > 2 * footprint_radius_) {
                Line line = new Line();
                line.setPoint(Vector2.scale(Vector2.normalize(rel_position), dist));
                line.setDir(Vector2.negative(Vector2.normalize(Vector2.minus(obst1, obst2))));
                addOrcaLines.add(line);
                return;

            }

            rel_position = Vector2.scale(Vector2.normalize(rel_position), Vector2.abs(rel_position) - dist / 2.0);

            Vector<Vector2> obst = new Vector<Vector2>();
            obst.add(Vector2.minus(obst1, position_obst));
            obst.add(Vector2.minus(obst2, position_obst));
            List<Vector2> mink_sum = Methods_Planners.minkowskiSumConvexHull(own_footprint, obst);

            Vector2 min = new Vector2();
            Vector2 max = new Vector2();
            double min_ang = 0.0;
            double max_ang = 0.0;

            for (int i = 0; i < mink_sum.size(); i++) {
                double angle = Vector2.angleBetween(rel_position, Vector2.plus(rel_position, mink_sum.get(i)));
                if (Methods_Planners.leftOf(new Vector2(0.0, 0.0), rel_position, Vector2.plus(rel_position, mink_sum.get(i)))) {
                    if (-angle < min_ang) {
                        min = Vector2.plus(rel_position, mink_sum.get(i));
                        min_ang = -angle;
                    }
                } else {
                    if (angle > max_ang) {
                        max = Vector2.plus(rel_position, mink_sum.get(i));
                        max_ang = angle;
                    }
                }
            }

            Line line = new Line();
            line.setPoint(Vector2.scale(Vector2.normalize(rel_position), dist / 2.0));
            if (Vector2.absSqr(Vector2.minus(position_obst, obst1)) > Vector2.absSqr(Vector2.minus(position_obst, obst2))) {
                // ROS_ERROR("max_ang = %.2f", max_ang);
                line.setDir(Vector2.rotateVectorByAngle(Vector2.normalize(max), 0.1));
            } else {
                // ROS_ERROR("min_ang = %.2f", min_ang);
                line.setDir(Vector2.rotateVectorByAngle(Vector2.normalize(min), 0.1));

            }
            addOrcaLines.add(line);

        }
    }

    void computeObstacleLine(Vector2 obst) {
        Line line = new Line();
        Vector2 relative_position = Vector2.minus(obst, position.getPos());
        double dist_to_footprint;
        double dist = Vector2.abs(Vector2.minus(position.getPos(), obst));
        if (!has_polygon_footprint_)
            dist_to_footprint = footprint_radius_;
        else {
            dist_to_footprint = getDistToFootprint(relative_position);
            if (dist_to_footprint == -1) {
                dist_to_footprint = footprint_radius_;
            }
        }
        dist = dist - dist_to_footprint - 0.03;

        line.setPoint(Vector2.scale(Vector2.normalize(relative_position), dist));
        line.setDir(new Vector2(-(Vector2.normalize(relative_position)).getY(), (Vector2.normalize(relative_position)).getX()));
        addOrcaLines.add(line);
    }

    double getDistToFootprint(Vector2 point) {
        Vector2 result;
        for (int i = 0; i < footprint_minkowski_lines.size(); i++) {
            LinePair l1 = new LinePair(footprint_minkowski_lines.get(i).getFirst(), footprint_minkowski_lines.get(i).getSecond());
            LinePair l2 = new LinePair(new Vector2(0.0, 0.0), point);

            result = LinePair.Intersection(l1, l2);
            if (result != null) {
                //ROS_DEBUG("Result = %f, %f, dist %f", result.x(), result.y(), collvoid::abs(result));
                return Vector2.abs(result);
            }
        }
        //ROS_DEBUG("Obstacle Point within Footprint. I am close to/in collision");
        return -1;
    }


    void computeClearpathVelocity(Vector2 pref_velocity) {
        //account for nh error
        neighbors_lock_.lock();
        try {
            radius += cur_allowed_error_;

            if (controlled) {
                computeAgentVOs();
            }
            newVelocity = ClearPathMethods.calculateClearpathVelocity(samples, voAgents, addOrcaLines, pref_velocity, max_speed_x_, useTruancation);

            radius -= cur_allowed_error_;
        } finally {
            neighbors_lock_.unlock();
        }
        //for visualization
//        msgPublisher.publishHoloSpeed(position, newVelocity, global_frame_, base_frame_, speed_pub_);
//        msgPublisher.publishVOs(position, voAgents, useTruancation, global_frame_, base_frame_, vo_pub_);
//        msgPublisher.publishPoints(position, samples, global_frame_, base_frame_, samples_pub_);
//        msgPublisher.publishOrcaLines(addOrcaLines, position, global_frame_, base_frame_, lines_pub_);


    }

    void computeAgentVOs() {
        // neighbors are published with localization uncertainty that means radius and footprint
        // include localization uncertainty radius and minkowski footprint.
        for (int i = 0; i < AgentNeighbors.size(); i++) {
            VO new_agent_vo;
            //use footprint or radius to create VO
            if (convex) {
                if (AgentNeighbors.get(i).controlled) {
                    new_agent_vo = ClearPathMethods.createVO(position.getPos(), footprint_rotated, velocity, AgentNeighbors.get(i).position.getPos(), AgentNeighbors.get(i).footprint_rotated, AgentNeighbors.get(i).velocity, voType);
                } else {
                    new_agent_vo = ClearPathMethods.createVO(position.getPos(), footprint_rotated, velocity, AgentNeighbors.get(i).position.getPos(), AgentNeighbors.get(i).footprint_rotated, AgentNeighbors.get(i).velocity, ClearPathMethods.VOS);
                }
            } else {
                if (AgentNeighbors.get(i).controlled) {
                    new_agent_vo = ClearPathMethods.createVO(position.getPos(), radius, velocity, AgentNeighbors.get(i).position.getPos(), AgentNeighbors.get(i).radius, AgentNeighbors.get(i).velocity, voType);
                } else {
                    new_agent_vo = ClearPathMethods.createVO(position.getPos(), radius, velocity, AgentNeighbors.get(i).position.getPos(), AgentNeighbors.get(i).radius, AgentNeighbors.get(i).velocity, ClearPathMethods.VOS);
                }
            }
            //truncation--not collide in certain amount of time
            if (useTruancation) {
                new_agent_vo = ClearPathMethods.createTruncVO(new_agent_vo, truncTime);
            }
            voAgents.add(new_agent_vo);
        }
    }

    public void stop() {
        rmqMsgManager.stop();
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

    public String getRobotId() {
        return Name;
    }

    public double getPublishMePeriod() {
        return publishMePeriod;
    }

    public long getLastTimeMePublished() {
        return lastTimeMePublished;
    }

    public RMQMsgManager getRmqMsgManager() {
        return rmqMsgManager;
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

    public Vector2 getHolo_velocity_() {
        return holo_velocity_;
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

    public void setHolo_velocity_(Vector2 holo_velocity_) {
        this.holo_velocity_ = holo_velocity_;
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

    public void setFootprint_rotated(List<Vector2> footprint_rotated) {
        this.footprint_rotated = footprint_rotated;
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
    public void setFootprint_minkowski(List<Vector2> footprint) {
        if (footprint.size() < 2) {
            logger.severe("The footprint specified has less than two nodes");
            return;
        }
        this.me_lock_.lock();
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
            this.me_lock_.unlock();
        }
    }

    /*+++++++++++++++++++++++++Set stuff end+++++++++++++++++++++++++*/

}

