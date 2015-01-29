package cgl.iotrobots.collavoid.iotTopology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.TimeDelayAnalysis.Constants;
import cgl.iotrobots.collavoid.commons.TimeDelayAnalysis.TimeDelayRecorder;
import cgl.iotrobots.collavoid.commons.planners.*;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;
import java.util.Map;
import java.util.Vector;

public class VOLinesComputeBolt_ extends BaseRichBolt {
    private Logger logger = LoggerFactory.getLogger(VOLinesComputeBolt_.class);
    private Agent agent;
    private OutputCollector collector;
    TimeDelayRecorder cmdDelayRecorder;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
        cmdDelayRecorder = new TimeDelayRecorder(
                Constants.COMPUTATION_DELAY,
                Constant_msg.KEY_VELOCITY_CMD,
                topologyContext.getThisComponentId());
        cmdDelayRecorder.open(false);
    }

    @Override
    public void execute(Tuple tuple) {
        cmdDelayRecorder.append(
                tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                tuple.getLongByField(Constant_storm.FIELDS.TIME_FIELD),
                System.currentTimeMillis());
        agent = (Agent) Utils.deserialize(tuple.getBinaryByField(Constant_storm.FIELDS.AGENT_FIELD));
        agent.addOrcaLines.clear();
        agent.voAgents.clear();
//        List<Line> ConstraintLines = new ArrayList<Line>();
        //List<VO> VOs = new ArrayList<VO>();
        if (!agent.holo_robot_)
            addNHConstraints(agent.addOrcaLines);
        Methods_Planners.NHORCA.addAccelerationConstraintsXY(
                agent.max_vel_x_,
                agent.acc_lim_x_,
                agent.max_vel_y_,
                agent.acc_lim_y_,
                agent.velocity,
                agent.position.getHeading(),
                agent.controlPeriod,
                agent.holo_robot_,
                agent.addOrcaLines
        );
        computeObstacleVOs(agent.voAgents, agent.addOrcaLines);
        if (agent.controlled)
            computeAgentVOs(agent.voAgents);
        collector.emit(new Values(tuple.getValue(0), tuple.getValue(1), Utils.serialize(agent)));
        collector.ack(tuple);
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.AGENT_FIELD
        ));
    }

    private void addNHConstraints(List<Line> nhConstLines) {
        double min_error = agent.minErrorHolo;
        double max_error = agent.maxErrorHolo;
        double error = max_error;
        double v_max_ang = vMaxAng();

        //ROS_ERROR("v_max_ang %.2f", v_max_ang);

        if (agent.min_dist < 2.0 * agent.footprint_radius_ + agent.cur_loc_unc_radius_) {
            error = (max_error - min_error) /
                    (Math.pow(2 * (agent.footprint_radius_ + agent.cur_loc_unc_radius_), 2)) *
                    Math.pow(agent.min_dist, 2) + min_error; // how much error do i allow?
            //ROS_DEBUG("Error = %f", error);
            if (agent.min_dist < 0) {
                error = min_error;
                // ROS_DEBUG("%s I think I am in collision", me_->getId().c_str());
            }
        }
        agent.cur_allowed_nh_error_ = 1.0 / 3.0 * agent.cur_allowed_nh_error_ + 2.0 / 3.0 * error;
        //ROS_ERROR("error = %f", cur_allowed_nh_error_);
        double speed_ang = Math.atan2(agent.prefVelociy.getY(), agent.prefVelociy.getX());
        double dif_ang = Methods_Planners.shortest_angular_distance(agent.position.getHeading(), speed_ang);
        //calculate possible tracking holomonic robot speed range
        if (Math.abs(dif_ang) > Math.PI / 2.0) { // || cur_allowed_nh_error_ < 2.0 * min_error) {
            double max_track_speed = Methods_Planners.NHORCA.calculateMaxTrackSpeedAngle(
                    agent.time_to_holo_,
                    Math.PI / 2.0,
                    agent.cur_allowed_nh_error_,
                    agent.max_vel_x_,
                    agent.max_vel_th_,
                    v_max_ang);
            // tracking errors are not in velocity space!!!!!!!!!!!!!!
            if (max_track_speed <= 2 * min_error) {
                max_track_speed = 2 * min_error;
            }
            Methods_Planners.NHORCA.addMovementConstraintsDiffSimple(
                    max_track_speed,
                    agent.position.getHeading(),
                    nhConstLines);
        } else {
            Methods_Planners.NHORCA.addMovementConstraintsDiff(
                    agent.cur_allowed_nh_error_,
                    agent.time_to_holo_,
                    agent.max_vel_x_,
                    agent.max_vel_th_,
                    agent.position.getHeading(),
                    v_max_ang,
                    nhConstLines);
        }
        agent.max_speed_x_ = vMaxAng();

    }

    double vMaxAng() {
        //double theoretical_max_v = max_vel_th_ * wheel_base_ / 2.0;
        //return theoretical_max_v - std::abs(base_odom_.twist.twist.angular.z) * wheel_base_/2.0;
        return agent.max_vel_x_; //TODO: fixme
    }

    void computeObstacleVOs(List<VO> obstacleVOs, List<Line> obstacleOrcaLines) {
        if (agent.obstacles_from_laser_.size() <= 0)
            return;
        double maxDist = Math.pow((Vector2.abs(agent.velocity) + 4.0 * agent.footprint_radius_), 2);
        for (Obstacle obstacle : agent.obstacles_from_laser_) {
            //why choose this limit?????????????????????????????????
            if (agent.use_obstacles_) {
                if (obstacle.getDistToAgent() < maxDist) {
                    if (agent.orca) {// currently set to false
                        createObstacleLine(obstacleOrcaLines, agent.footprint_original, obstacle.getBegin(), obstacle.getEnd());
                    } else {//called from CP
                        VO obstacle_vo = Methods_Planners.ClearPath.createObstacleVO(
                                agent.position.getPos(),
                                agent.footprint_radius_,
                                agent.footprint_original,
                                obstacle.getBegin(),
                                obstacle.getEnd()
                        );
                        obstacle_vo.setType("obstacleVo");
                        obstacleVOs.add(obstacle_vo);
                    }
                }
            }

        }
    }

    private void createObstacleLine(List<Line> obstacleOrcaLines, List<Vector2> own_footprint, Vector2 obst1, Vector2 obst2) {

        double dist = Methods_Planners.distSqPointLineSegment(obst1, obst2, agent.position.getPos());

        if (dist == Vector2.absSqr(Vector2.minus(agent.position.getPos(), obst1))) {
            computeObstacleLine(obstacleOrcaLines, obst1);
        } else if (dist == Vector2.absSqr(Vector2.minus(agent.position.getPos(), obst2))) {
            computeObstacleLine(obstacleOrcaLines, obst2);
        } else {
            Vector2 position_obst = Methods_Planners.projectPointOnLine(obst1, Vector2.minus(obst2, obst1), agent.position.getPos());
            Vector2 rel_position = Vector2.minus(position_obst, agent.position.getPos());
            dist = Math.sqrt(dist);
            double dist_to_footprint = getDistToFootprint(rel_position);
            if (dist_to_footprint == -1) {
                dist_to_footprint = agent.footprint_radius_;
            }
            dist = dist - dist_to_footprint - 0.03;

            if (dist < 0.0) {
                Line line = new Line();
                line.setPoint(Vector2.scale(Vector2.normalize(rel_position), (dist - 0.02)));
                line.setDir(Vector2.normalize(Vector2.minus(obst1, obst2)));
                obstacleOrcaLines.add(line);
                return;
            }

            if (Vector2.abs(Vector2.minus(agent.position.getPos(), obst1)) > 2 * agent.footprint_radius_ &&
                    Vector2.abs(Vector2.minus(agent.position.getPos(), obst2)) > 2 * agent.footprint_radius_) {
                Line line = new Line();
                line.setPoint(Vector2.scale(Vector2.normalize(rel_position), dist));
                line.setDir(Vector2.negative(Vector2.normalize(Vector2.minus(obst1, obst2))));
                obstacleOrcaLines.add(line);
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
                line.setDir(Vector2.rotateVectorByAngle(Vector2.normalize(max), 0.1));
            } else {
                line.setDir(Vector2.rotateVectorByAngle(Vector2.normalize(min), 0.1));

            }
            obstacleOrcaLines.add(line);

        }
    }

    private void computeObstacleLine(List<Line> obstacleOrcaLines, Vector2 obst) {
        Line line = new Line();
        Vector2 relative_position = Vector2.minus(obst, agent.position.getPos());
        double dist_to_footprint;
        double dist = Vector2.abs(Vector2.minus(agent.position.getPos(), obst));
        if (!agent.has_polygon_footprint_)
            dist_to_footprint = agent.footprint_radius_;
        else {
            dist_to_footprint = getDistToFootprint(relative_position);
            if (dist_to_footprint == -1) {
                dist_to_footprint = agent.footprint_radius_;
            }
        }
        dist = dist - dist_to_footprint - 0.03;

        line.setPoint(Vector2.scale(Vector2.normalize(relative_position), dist));
        line.setDir(new Vector2(-(Vector2.normalize(relative_position)).getY(),
                (Vector2.normalize(relative_position)).getX()));
        obstacleOrcaLines.add(line);
    }

    private double getDistToFootprint(Vector2 point) {
        Vector2 result;
        for (int i = 0; i < agent.footprint_minkowski_lines.size(); i++) {
            LinePair l1 = new LinePair(agent.footprint_minkowski_lines.get(i).getFirst(),
                    agent.footprint_minkowski_lines.get(i).getSecond());
            LinePair l2 = new LinePair(new Vector2(0.0, 0.0), point);

            result = LinePair.Intersection(l1, l2);
            if (result != null) {
                return Vector2.abs(result);
            }
        }
        return -1;
    }

    private void computeAgentVOs(List<VO> voAgents) {
        double radiusWithError = agent.radius + agent.cur_allowed_nh_error_;
        // neighbors are published with localization uncertainty that means radius and footprint
        // include localization uncertainty radius and minkowski footprint.
        for (Neighbor neighbor : agent.AgentNeighbors) {
            VO new_agent_vo;
            //use footprint or radius to create VO
            if (agent.convex) {
                if (neighbor.controlled) {
                    new_agent_vo = Methods_Planners.ClearPath.createVO(
                            agent.position.getPos(),
                            agent.footprint_rotated,
                            agent.velocity,
                            neighbor.position.getPos(),
                            neighbor.footprint_rotated,
                            neighbor.velocity,
                            agent.voType);
                } else {
                    new_agent_vo = Methods_Planners.ClearPath.createVO(
                            agent.position.getPos(),
                            agent.footprint_rotated,
                            agent.velocity,
                            neighbor.position.getPos(),
                            neighbor.footprint_rotated,
                            neighbor.velocity,
                            Methods_Planners.ClearPath.VOS);
                }
            } else {
                if (neighbor.controlled) {
                    new_agent_vo = Methods_Planners.ClearPath.createVO(
                            agent.position.getPos(),
                            radiusWithError,
                            agent.velocity,
                            neighbor.position.getPos(),
                            neighbor.radius,
                            neighbor.velocity,
                            agent.voType);
                } else {
                    new_agent_vo = Methods_Planners.ClearPath.createVO(
                            agent.position.getPos(),
                            radiusWithError,
                            agent.velocity,
                            neighbor.position.getPos(),
                            neighbor.radius,
                            neighbor.velocity,
                            Methods_Planners.ClearPath.VOS);
                }
            }
            //truncation--not collide in certain amount of time
            if (agent.useTruancation) {
                new_agent_vo = Methods_Planners.ClearPath.createTruncVO(new_agent_vo, agent.truncTime);
                new_agent_vo.setType("agentVo");
            }
            voAgents.add(new_agent_vo);
        }
    }
}
