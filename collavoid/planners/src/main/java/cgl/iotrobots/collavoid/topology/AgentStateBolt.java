package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.planners.AgentState;
import cgl.iotrobots.collavoid.commons.planners.Neighbor;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import com.esotericsoftware.kryo.Kryo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class AgentStateBolt extends BaseRichBolt {
    Logger logger = LoggerFactory.getLogger(AgentStateBolt.class);
    OutputCollector collector;
    Kryo kryo;
    Map<String, Double> truncTimeMap = new ConcurrentHashMap<String, Double>();
    Map<String, PoseShareMsg_> poseShareMsgMap = new ConcurrentHashMap<String, PoseShareMsg_>();
    Map<String, Odometry_> odometryMap = new ConcurrentHashMap<String, Odometry_>();
    Map<String, PoseShareMsg_> neighborShareMsgMap = new ConcurrentHashMap<String, PoseShareMsg_>();
    Map<String, PointCloud2_> scanMap = new ConcurrentHashMap<String, PointCloud2_>();
    Map<String, PoseArray_> poseArrayMap = new ConcurrentHashMap<String,PoseArray_>();
    long arrTime;
    long finishTime;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
        kryo = Methods_RMQ.getKryo();
    }

    @Override
    public void execute(Tuple tuple) {
        arrTime = System.currentTimeMillis();
        if (tuple.getSourceStreamId().equals(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM)) {
            emitAgentState(tuple);
        } else if (tuple.getSourceStreamId().equals(Constant_storm.Streams.PUBLISHME_STREAM)) {
            emitPoseShare(tuple);
        } else if (tuple.getSourceComponent().equals(Constant_storm.Components.ODOMETRY_SPOUT_COMPONENT)) {
            cacheOdometry(tuple);
        } else if (tuple.getSourceComponent().equals(Constant_storm.Components.POSE_SHARE_COMPONENT)) {
            cacheNeighborPoseShareMsg(tuple);
        } else if (tuple.getSourceComponent().equals(Constant_storm.Components.SCAN_COMPONENT)) {
            cacheScan(tuple);
        } else if (tuple.getSourceComponent().equals(Constant_storm.Components.POSE_ARRAY_COMPONENT)) {
            cachePoseArray(tuple);
        } else if (tuple.getSourceComponent().equals(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT)) {
            cachePoseShareMsg(tuple);
            cacheTruncTime(tuple);
        }

        collector.ack(tuple);
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM, new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.AGENT_STATE_FIELD,
                Constant_storm.FIELDS.EMIT_TIME_FIELD,
                Constant_storm.FIELDS.AGENT_IDX_FIELD
        ));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.PUBLISHME_PUB_STREAM, new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.POSE_SHARE_FIELD
        ));
        // back to dispatcher
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.PUBLISHME_STREAM, new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.EMIT_TIME_FIELD,
                Constant_storm.FIELDS.AGENT_IDX_FIELD
        ));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.ACK_STREAM, new Fields(
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.SEQUENCE_FIELD,
                Constant_storm.FIELDS.AGENT_IDX_FIELD
        ));
    }

    private void emitPoseShare(Tuple tuple) {
        PoseShareMsg_ msg;
        String id = tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
        String times = tuple.getStringByField(Constant_storm.FIELDS.EMIT_TIME_FIELD);
        times = times + arrTime + ",";
        if (poseShareReady(id)) {
            msg = poseShareMsgMap.get(id);
            Odometry_ odometry_ = odometryMap.get(msg.getId());
            msg.getHeader().setStamp(odometry_.getHeader().getStamp());
            msg.setPose(odometry_.getPose().copy());
            msg.setTwist(odometry_.getTwist().copy());
            if (poseArrayMap.containsKey(msg.getId())) {
                if (msg.getFootprint_original().size() == 0) {
                    logger.error("Original foot print zero!");
                    throw new RuntimeException("Original foot print zero!");
                }
                msg.setFootPrint_Minkowski(
                        Methods.getMinkowskiFootprint(msg.getFootprint_original(), poseArrayMap.get(msg.getId())));
            }
            collector.emit(Constant_storm.Streams.PUBLISHME_PUB_STREAM, new Values(
                    tuple.getValue(0),
                    tuple.getValue(1),
                    Methods_RMQ.serialize(kryo, msg)
            ));
        }
        finishTime = System.currentTimeMillis();
        collector.emit(Constant_storm.Streams.PUBLISHME_STREAM, new Values(
                tuple.getValue(0),
                tuple.getValue(1),
                times + finishTime + ",",
                tuple.getValueByField(Constant_storm.FIELDS.AGENT_IDX_FIELD)
        ));
    }

    private boolean poseShareReady(String id) {
        if (odometryMap.containsKey(id) && poseShareMsgMap.containsKey(id))
            return true;
        return false;
    }

    private void emitAgentState(Tuple tuple) {
        String sensorID = tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);

        String times = tuple.getStringByField(Constant_storm.FIELDS.EMIT_TIME_FIELD);
        times = times + arrTime + ",";

        AgentState agentState = null;

        if (AgentStateReady(sensorID)) {

            agentState = new AgentState(sensorID);
            //get odometry
            agentState.odometry_ = odometryMap.get(sensorID);

            //get neighbors
            agentState.neighbors = getNeighbors(sensorID);

            //get obstacles
            if (scanMap.containsKey(sensorID)) {
                agentState.obstacles = Methods.getObstacles(agentState.neighbors,
                        poseShareMsgMap.get(sensorID).getRadius(),
                        scanMap.get(sensorID));
            }

            //get minkowski footprint
            if (poseArrayMap.containsKey(sensorID)) {
                agentState.minkowskiFootprint = Methods.getMinkowskiFootprint(
                        poseShareMsgMap.get(sensorID).getFootprint_original(),
                        poseArrayMap.get(sensorID));
            }
        }
        finishTime = System.currentTimeMillis();
        collector.emit(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM, new Values(
                tuple.getValue(0),
                tuple.getValue(1),
                agentState,
                times + finishTime + ",",
                tuple.getValueByField(Constant_storm.FIELDS.AGENT_IDX_FIELD)
        ));
    }

    private boolean AgentStateReady(String sensorID) {
        if (odometryMap.containsKey(sensorID) &&
                poseShareMsgMap.containsKey(sensorID))
            return true;
        return false;
    }

    private List<Neighbor> getNeighbors(String id) {
        List<Neighbor> neighbors = new ArrayList<Neighbor>();
        Odometry_ odom = odometryMap.get(id);
        double trunctime = truncTimeMap.get(id);
        double radius = poseShareMsgMap.get(id).getRadius();

        for (Map.Entry<String, PoseShareMsg_> e : neighborShareMsgMap.entrySet()) {
            if (!e.getKey().equals(id)) {
                if (Methods.isCloseNeighbor(odom, radius, e.getValue(), trunctime)) {
                    neighbors.add(Methods.extractNeighbor(e.getValue()));
                }
            }
        }
        return neighbors;
    }

    private void cacheTruncTime(Tuple tuple) {
        truncTimeMap.put(tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                tuple.getDoubleByField(Constant_storm.FIELDS.TRUNC_TIME_FIELD));
    }

    private void cachePoseShareMsg(Tuple tuple) {
        String sensorID = tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
        PoseShareMsg_ msg = (PoseShareMsg_) tuple.getValueByField(Constant_storm.FIELDS.POSE_SHARE_FIELD);
        poseShareMsgMap.put(sensorID, msg);
        collector.emit(Constant_storm.Streams.ACK_STREAM, new Values(
                sensorID,
                msg.getHeader().getSeq(),
                tuple.getValueByField(Constant_storm.FIELDS.AGENT_IDX_FIELD)));
    }

    private void cachePoseArray(Tuple tuple) {
        poseArrayMap.put(tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                (PoseArray_) tuple.getValueByField(Constant_storm.FIELDS.POSE_ARRAY_FIELD));
    }

    private void cacheScan(Tuple tuple) {
        scanMap.put(tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                (PointCloud2_) tuple.getValueByField(Constant_storm.FIELDS.SCAN_FIELD));
    }

    private void cacheNeighborPoseShareMsg(Tuple tuple) {
        neighborShareMsgMap.put(tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                (PoseShareMsg_) tuple.getValueByField(Constant_storm.FIELDS.POSE_SHARE_FIELD));
    }

    private void cacheOdometry(Tuple tuple) {
        Odometry_ odometry_ = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        double vx = odometry_.getTwist().getLinear().getX();
        double vy = odometry_.getTwist().getLinear().getY();
        double vxTransformed = Math.sqrt(vx * vx + vy * vy);
        odometry_.getTwist().getLinear().setX(vxTransformed);
        odometry_.getTwist().getLinear().setY(0.0);
        odometryMap.put(tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD), odometry_);
    }
}
