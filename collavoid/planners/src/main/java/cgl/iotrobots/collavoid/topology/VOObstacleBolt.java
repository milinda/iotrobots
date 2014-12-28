package cgl.iotrobots.collavoid.topology;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.planners.*;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class VOObstacleBolt extends BaseBasicBolt {
    private Agent agent;
    private int seq = 0;
    private List<VO> obstacleVOs = new ArrayList<VO>();
    private List<Line> obstacleOrcaLines = new ArrayList<Line>();

    @Override
    public void execute(Tuple input, BasicOutputCollector collector) {
        agent = (Agent) input.getValueByField(Constant_storm.FIELDS.AGENT_FIELD);
        obstacleOrcaLines.clear();
        obstacleVOs.clear();
        computeObstacleVOs();
        collector.emit(new Values(
                input.getValue(0),
                input.getValue(1),
                obstacleVOs,
                obstacleOrcaLines,
                seq++));

    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.OBSTACLE_VO_FIELD,
                Constant_storm.FIELDS.OBSTACLE_LINES_FIELD,
                Constant_storm.FIELDS.SEQUENCE_FIELD
        ));
    }

    void computeObstacleVOs() {
        if (agent.obstacles_from_laser_.size() <= 0)
            return;
        double maxDist = Math.pow((Vector2.abs(agent.velocity) + 4.0 * agent.footprint_radius_), 2);
        for (Obstacle obstacle : agent.obstacles_from_laser_) {
            //why choose this limit?????????????????????????????????
            if (agent.use_obstacles_) {
                if (obstacle.getDistToAgent() < maxDist) {
                    if (agent.orca) {// currently set to false
                        createObstacleLine(agent.footprint_original, obstacle.getBegin(), obstacle.getEnd());
                    } else {//called from CP
                        VO obstacle_vo = Methods_Planners.ClearPath.createObstacleVO(
                                agent.position.getPos(),
                                agent.footprint_radius_,
                                agent.footprint_original,
                                obstacle.getBegin(),
                                obstacle.getEnd()
                        );
                        obstacleVOs.add(obstacle_vo);
                    }
                }
            }

        }
    }

    private void createObstacleLine(List<Vector2> own_footprint, Vector2 obst1, Vector2 obst2) {

        double dist = Methods_Planners.distSqPointLineSegment(obst1, obst2, agent.position.getPos());

        if (dist == Vector2.absSqr(Vector2.minus(agent.position.getPos(), obst1))) {
            computeObstacleLine(obst1);
        } else if (dist == Vector2.absSqr(Vector2.minus(agent.position.getPos(), obst2))) {
            computeObstacleLine(obst2);
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

    private void computeObstacleLine(Vector2 obst) {
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
        for (int i = 0; i < agent.footprint_original_lines.size(); i++) {
            LinePair l1 = new LinePair(agent.footprint_original_lines.get(i).getFirst(),
                    agent.footprint_original_lines.get(i).getSecond());
            LinePair l2 = new LinePair(new Vector2(0.0, 0.0), point);

            result = LinePair.Intersection(l1, l2);
            if (result != null) {
                return Vector2.abs(result);
            }
        }
        return -1;
    }
}
