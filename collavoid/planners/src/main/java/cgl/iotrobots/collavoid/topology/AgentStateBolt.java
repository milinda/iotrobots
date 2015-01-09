package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.Neighbor;
import cgl.iotrobots.collavoid.commons.planners.Obstacle;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.PoseShareMsg_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AgentStateBolt extends BaseRichBolt {
    private Odometry_ odometry_ = null;
    private List<Vector2> MinkowskiFootprint = new ArrayList<Vector2>();
    private List<Obstacle> obstacles = new ArrayList<Obstacle>();
    private Map<String, Neighbor> Agents = new HashMap<String, Neighbor>();
    private String sensorID = null;
    private OutputCollector collector;
    private Logger logger = LoggerFactory.getLogger(AgentStateBolt.class);
    private boolean minkowskiFootprintChanged = false;
    
    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        this.collector = collector;
    }

    @Override
    public void execute(Tuple tuple) {
        if (null == sensorID) {
            if (tuple.contains(Constant_storm.FIELDS.SENSOR_ID_FIELD)) {
                sensorID = (String) tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            } else {
                collector.ack(tuple);
                return;
            }
        }

        String sourceComp = tuple.getSourceComponent();
        if (sourceComp.equals(Constant_storm.Components.ODOMETRY_TRANSFORM_COMPONENT)) {
            odometry_ = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        } else if (sourceComp.equals(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT)) {
            Agents = (Map<String, Neighbor>) Utils.deserialize(
                    tuple.getBinaryByField(Constant_storm.FIELDS.ALL_AGENTS_FIELD));
        } else if (tuple.getSourceStreamId().equals(Constant_storm.Streams.PREFERRED_VELOCITY_STREAM)) {
            updatePrefVel(tuple);
        } else if (tuple.getSourceStreamId().equals(Constant_storm.Streams.PUBLISH_ME_TIMER_STREAM)) {
            updateMeState(tuple);
        } else if (sourceComp.equals(Constant_storm.Components.GET_OBSTACLES_COMPONENT)) {
            obstacles = (List<Obstacle>) tuple.getValueByField(Constant_storm.FIELDS.OBSTACLE_FIELD);
        } else if (sourceComp.equals(Constant_storm.Components.GET_MINKOWSKI_FOOTPRINT_COMPONENT)) {
            MinkowskiFootprint = (List<Vector2>) tuple.getValueByField(Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD);
            minkowskiFootprintChanged = true;
        } else if (tuple.getSourceStreamId().equals(Constant_storm.Streams.RESET_STREAM)) {
            odometry_ = null;
            minkowskiFootprintChanged = false;
        }
        collector.ack(tuple);
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.PUBLISHME_STREAM, new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.ODOMETRY_FIELD,
                Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD

        ));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM, new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.ODOMETRY_FIELD,
                Constant_storm.FIELDS.NEIGHBORS_FIELD,
                Constant_storm.FIELDS.OBSTACLE_FIELD,
                Constant_storm.FIELDS.PREFERRED_VELOCITY_FIELD,
                Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD
        ));
    }

    private void updateMeState(Tuple input) {
        Values pubValue = new Values();
        pubValue.add(input.getValueByField(Constant_storm.FIELDS.TIME_FIELD));
        pubValue.add(sensorID);
        if (odometry_ != null) {
            pubValue.add(odometry_.copy());
        } else
            pubValue.add(null);    // odometry may be null, it is left to agentbolt to check
        if (minkowskiFootprintChanged) {
            pubValue.add(getMinkowskiFootprint(MinkowskiFootprint));
            minkowskiFootprintChanged = false;
        } else {
            pubValue.add(null);
        }
        collector.emit(Constant_storm.Streams.PUBLISHME_STREAM, pubValue);
    }

    private void updatePrefVel(Tuple input) {
        Values velValue = new Values();
        Odometry_ odom = (Odometry_) input.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        Vector2 prevel = (Vector2) input.getValueByField(Constant_storm.FIELDS.PREFERRED_VELOCITY_FIELD);

        velValue.add(input.getValueByField(Constant_storm.FIELDS.TIME_FIELD));
        velValue.add(input.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD));
        velValue.add(odom.copy());
        velValue.add(getNeighbors(Agents, sensorID));
        velValue.add(getObstacles(obstacles));
        velValue.add(prevel.copy());
        if (minkowskiFootprintChanged) {
            velValue.add(getMinkowskiFootprint(MinkowskiFootprint));//may not need to copy
            minkowskiFootprintChanged = false;
        } else {
            velValue.add(null);
        }

        collector.emit(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM, velValue);

    }

//    private void updateAgentList(PoseShareMsg_ msg) {
//        Neighbor newagent = new Neighbor(msg.getName());
//        newagent.holo_robot_ = msg.getHoloRobot();
//        newagent.base_odom_=new Odometry_();
//        newagent.base_odom_.setPose(msg.getPose());
//        newagent.base_odom_.setTwist(msg.getTwist());
//        newagent.radius = msg.getRadius();
//        newagent.controlled = msg.getControlled();
//        newagent.setControlPeriod(msg.getControlPeriod());
//        newagent.setFootprint_minkowski(msg.getFootPrint_Minkowski());
//        newagent.last_seen_ = msg.getHeader().getStamp();
//        Agents.put(msg.getName(), newagent);
//    }

    private List<Neighbor> getNeighbors(Map<String, Neighbor> agents, String sensorID) {
        List<Neighbor> neighbors = new ArrayList<Neighbor>();
        for (Map.Entry<String, Neighbor> agt : agents.entrySet()) {
            if (agt.getKey().equals(sensorID))
                continue;
            neighbors.add(agt.getValue().copy());
        }
        return neighbors;
    }

    private List<Obstacle> getObstacles(List<Obstacle> obstacleList) {
        List<Obstacle> res = new ArrayList<Obstacle>();
        for (Obstacle obs : obstacleList) {
            res.add(obs.copy());
        }
        return res;
    }

    private List<Vector2> getMinkowskiFootprint(List<Vector2> minkowskiFootprint) {
        List<Vector2> minfootprint = new ArrayList<Vector2>(minkowskiFootprint.size());
        for (Vector2 pt : minkowskiFootprint) {
            minfootprint.add(new Vector2(pt.getX(), pt.getY()));
        }
        return minfootprint;
    }

}
