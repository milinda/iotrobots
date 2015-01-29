package cgl.iotrobots.collavoid.topologyStreaming;

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
import cgl.iotrobots.collavoid.commons.planners.Neighbor;
import cgl.iotrobots.collavoid.commons.planners.Obstacle;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AgentStateBolt extends BaseRichBolt {
    private Logger logger = LoggerFactory.getLogger(AgentStateBolt.class);
    private OutputCollector collector;
    private Map<String, Neighbor> Agents = new HashMap<String, Neighbor>();
    private Map<String, AgentStateContext> contexts = new HashMap<String, AgentStateContext>();
    private AgentStateContext currentContext;
    private String id;
    TimeDelayRecorder odomDelayRecorder, scanDelayRecorder, poseShareDelayRecorder, poseArrayDelayRecorder, cmdDelayRecorder;

    private class AgentStateContext {
        String sensorID = null;
        Odometry_ odometry_ = null;
        List<Vector2> MinkowskiFootprint = new ArrayList<Vector2>();
        List<Obstacle> obstacles = new ArrayList<Obstacle>();
        boolean minkowskiFootprintChanged = false;

        public AgentStateContext(String id) {
            sensorID = id;
        }
    }

    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        this.collector = collector;
        odomDelayRecorder = new TimeDelayRecorder(
                Constants.PARAMETER_DELAY,
                Constant_msg.KEY_ODOMETRY,
                context.getThisComponentId());
        odomDelayRecorder.open(false);
        scanDelayRecorder = new TimeDelayRecorder(
                Constants.PARAMETER_DELAY,
                Constant_msg.KEY_SCAN,
                context.getThisComponentId());
        scanDelayRecorder.open(false);
        poseShareDelayRecorder = new TimeDelayRecorder(
                Constants.PARAMETER_DELAY,
                Constant_msg.KEY_POSE_SHARE,
                context.getThisComponentId());
        poseShareDelayRecorder.open(false);
        poseArrayDelayRecorder = new TimeDelayRecorder(
                Constants.PARAMETER_DELAY,
                Constant_msg.KEY_POSE_ARRAY,
                context.getThisComponentId());
        poseArrayDelayRecorder.open(false);
        cmdDelayRecorder = new TimeDelayRecorder(
                Constants.COMPUTATION_DELAY,
                Constant_msg.KEY_VELOCITY_CMD,
                context.getThisComponentId());
        cmdDelayRecorder.open(false);
    }

    @Override
    public void execute(Tuple tuple) {
        if (tuple.contains(Constant_storm.FIELDS.SENSOR_ID_FIELD)) {
            id = (String) tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            if (contexts.get(id) == null) {
                contexts.put(id, new AgentStateContext(id));
            }
            currentContext = contexts.get(id);
        }

        String sourceComp = tuple.getSourceComponent();
        if (sourceComp.equals(Constant_storm.Components.ODOMETRY_TRANSFORM_COMPONENT)) {
            odomDelayRecorder.append(
                    tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                    tuple.getLongByField(Constant_storm.FIELDS.TIME_FIELD),
                    System.currentTimeMillis());
            currentContext.odometry_ = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        } else if (sourceComp.equals(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT)) {
            Agents = (Map<String, Neighbor>) Utils.deserialize(
                    tuple.getBinaryByField(Constant_storm.FIELDS.ALL_AGENTS_FIELD));
        } else if (tuple.getSourceStreamId().equals(Constant_storm.Streams.PREFERRED_VELOCITY_STREAM)) {
            cmdDelayRecorder.append(
                    tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                    tuple.getLongByField(Constant_storm.FIELDS.TIME_FIELD),
                    System.currentTimeMillis());
            updatePrefVel(tuple);
        } else if (tuple.getSourceStreamId().equals(Constant_storm.Streams.PUBLISH_ME_TIMER_STREAM)) {
            poseShareDelayRecorder.append(
                    tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                    tuple.getLongByField(Constant_storm.FIELDS.TIME_FIELD),
                    System.currentTimeMillis());
            updateMeState(tuple);
        } else if (sourceComp.equals(Constant_storm.Components.GET_OBSTACLES_COMPONENT)) {
            scanDelayRecorder.append(
                    tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                    tuple.getLongByField(Constant_storm.FIELDS.TIME_FIELD),
                    System.currentTimeMillis());
            currentContext.obstacles = (List<Obstacle>) tuple.getValueByField(Constant_storm.FIELDS.OBSTACLE_FIELD);
        } else if (sourceComp.equals(Constant_storm.Components.GET_MINKOWSKI_FOOTPRINT_COMPONENT)) {
            poseArrayDelayRecorder.append(
                    tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                    tuple.getLongByField(Constant_storm.FIELDS.TIME_FIELD),
                    System.currentTimeMillis());
            currentContext.MinkowskiFootprint = (List<Vector2>) tuple.getValueByField(Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD);
            currentContext.minkowskiFootprintChanged = true;
        } else if (tuple.getSourceStreamId().equals(Constant_storm.Streams.RESET_STREAM)) {
            currentContext.odometry_ = null;
            currentContext.minkowskiFootprintChanged = false;
            collector.emit(Constant_storm.Streams.RESET_STREAM, tuple.getValues());
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
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.RESET_STREAM, new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.RESET_FIELD
        ));
    }

    private void updateMeState(Tuple input) {
        Values pubValue = new Values();
        pubValue.add(input.getValueByField(Constant_storm.FIELDS.TIME_FIELD));
        pubValue.add(currentContext.sensorID);
        if (currentContext.odometry_ != null) {
            pubValue.add(currentContext.odometry_.copy());
        } else
            pubValue.add(null);    // odometry may be null, it is left to agentbolt to check

        //update only when footprint changed
        if (currentContext.minkowskiFootprintChanged) {
            pubValue.add(getMinkowskiFootprint(currentContext.MinkowskiFootprint));
            currentContext.minkowskiFootprintChanged = false;
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
        velValue.add(getNeighbors(Agents, currentContext.sensorID));
        velValue.add(getObstacles(currentContext.obstacles));
        velValue.add(prevel.copy());
        if (currentContext.minkowskiFootprintChanged) {
            velValue.add(getMinkowskiFootprint(currentContext.MinkowskiFootprint));//may not need to copy
            currentContext.minkowskiFootprintChanged = false;
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
