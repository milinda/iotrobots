package cgl.iotrobots.collavoid.topology;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.planners.*;
import cgl.iotrobots.collavoid.commons.rmqmsg.PointCloud2_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Vector3d_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class GetObstacleBolt extends BaseBasicBolt {
    private Logger logger = LoggerFactory.getLogger(GetObstacleBolt.class);
    private Map<String, Neighbor> agents = new HashMap<String, Neighbor>();
    private String sensorID = null;//use sensor Id as robot id

    @Override
    public void execute(Tuple tuple, BasicOutputCollector basicOutputCollector) {
        if (tuple.contains(Constant_storm.FIELDS.SENSOR_ID_FIELD)) {
            sensorID = tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
        }
        if (tuple.getSourceComponent().equals(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT)) {
            agents = (Map<String, Neighbor>) Utils.deserialize(tuple.getBinaryByField(Constant_storm.FIELDS.ALL_AGENTS_FIELD));
        } else if (tuple.getSourceComponent().equals(Constant_storm.Components.SCAN_COMPONENT)) {
            List<Object> emit = new ArrayList<Object>();
            List<Obstacle> obstacles = new ArrayList<Obstacle>();
            PointCloud2_ pointCloud2_ = (PointCloud2_) tuple.getValueByField(Constant_storm.FIELDS.SCAN_FIELD);
            getObstacles(pointCloud2_, obstacles);
            emit.add(tuple.getValue(0));
            emit.add(sensorID);
            emit.add(obstacles);
            basicOutputCollector.emit(emit);
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.OBSTACLE_FIELD
        ));
    }

    private void getObstacles(PointCloud2_ msg, List<Obstacle> obstacles) {
        if (msg.getWidth() * msg.getHeight() == 0) {
            obstacles.clear();
            return;
        }
        if (msg.getData().length % 3 != 0) {
            logger.error("Bad PointCloud2_ data!!");
            return;
        }
        List<Vector3d_> point3ds = new ArrayList<Vector3d_>();

        double[] data = msg.getData();
        int pn = data.length / 3;
        int idx = 0;
        int idx_;
        while (idx < pn) {
            idx_ = idx * 3;
            point3ds.add(new Vector3d_(data[idx_], data[idx_ + 1], data[idx_ + 2]));
            idx++;
        }
        obstacles.clear();
        double threshold_convex = 0.03;
        double threshold_concave = -0.03;
        //    ROS_ERROR("%d", (int)cloud.points.size());
        Vector2 start;
        for (int i = 0; i < point3ds.size(); i++) {
            start = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
            while (pointInNeighbor(start) && i < point3ds.size() - 1) {
                i++;
                start = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
            }
            if (i == point3ds.size()) {
                //it is a agent
                return;
            }

            boolean found = false;
            Vector2 prev = new Vector2(start);
            double first_ang = 0;
            double prev_ang = 0;
            Vector2 next;
            while (!found) {
                i++;
                if (i == point3ds.size()) {
                    break;
                }
                next = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
                while (pointInNeighbor(next) && i < point3ds.size() - 1) {
                    i++;
                    next = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
                }

                if (Vector2.abs(Vector2.minus(next, prev)) > 2 * Parameters.FOOTPRINT_RADIUS) {
                    found = true;
                    break;
                }
                Vector2 dif = Vector2.minus(next, start);
                double ang = Math.atan2(dif.getY(), dif.getX());
                if (!prev.equals(start)) {
                    if (ang - first_ang < threshold_concave) {
                        found = true;
                        i -= 2;
                        break;
                    }
                    if (ang - prev_ang < threshold_concave) {
                        found = true;
                        i -= 2;
                        break;
                    }
                    if (ang - prev_ang > threshold_convex) { //going towards me
                        found = true;
                        i -= 2;
                        break;
                    }
                    if (ang - first_ang > threshold_convex) { //going towards me
                        found = true;
                        i -= 2;
                        break;
                    }

                } else {
                    first_ang = ang;
                }
                prev = new Vector2(next);
                prev_ang = ang;
            }
            if (!start.equals(prev)) {
                Obstacle obst = new Obstacle(start, prev);
                obstacles.add(obst);
            }
        }

    }

    private boolean pointInNeighbor(Vector2 point) {
        double dist;
        for (Map.Entry<String, Neighbor> e : agents.entrySet()) {
            if (e.getKey().equals(sensorID))
                continue;
                dist = Vector2.abs(Vector2.minus(point, e.getValue().position.getPos()));
                if (dist <= e.getValue().radius) {
                    return true;
                }
        }
        return false;
    }

}
