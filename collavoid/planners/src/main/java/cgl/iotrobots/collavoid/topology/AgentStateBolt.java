package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.Obstacle;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class AgentStateBolt extends BaseRichBolt {
    private Odometry_ odometry_ = null;
    private List<Obstacle> obstacles = new ArrayList<Obstacle>();
    private List<Agent> neighbors = new ArrayList<Agent>();
    private List<Vector2> Footprint_minkowski = new ArrayList<Vector2>();
    private Vector2 prefVel = null;
    private String sensorID = null;
    private OutputCollector collector;
    private Tuple input;

    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        this.collector = collector;
    }

    @Override
    public void execute(Tuple tuple) {
        String sourceComp = tuple.getSourceComponent();
        input = tuple;
        if (sourceComp.equals(Constant_storm.Components.ODOMETRY_COMPONENT)) {
            odometry_ = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
            if (null == sensorID) {
                sensorID = (String) tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            }
        } else if (sourceComp.equals(Constant_storm.Components.GET_OBSTACLES_COMPONENT))
            obstacles = (List<Obstacle>) tuple.getValueByField(Constant_storm.FIELDS.OBSTACLE_FIELD);
        else if (sourceComp.equals(Constant_storm.Components.GET_ALL_AGENTS_COMPONENT)) {
            Map<String, Agent> agents = (Map<String, Agent>) tuple.getValueByField(
                    Constant_storm.FIELDS.ALL_AGENTS_FIELD);
            updateNeighbors(agents);
        } else if (sourceComp.equals(Constant_storm.Components.GET_MINKOWSKI_FOOTPRINT_COMPONENT))
            Footprint_minkowski = (List<Vector2>) tuple.getValueByField(Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD);
        else if (tuple.getSourceStreamId().equals(Constant_storm.Streams.PREFERRED_VELOCITY_STREAM)) {
            prefVel = (Vector2) tuple.getValueByField(Constant_storm.FIELDS.PREFERRED_VELOCITY_FIELD);
            snedPrefvel();
        } else if (tuple.getSourceStreamId().equals(Constant_storm.Streams.PUBLISH_ME_TIMER_STREAM))
            sendStateToPub();

        collector.ack(tuple);

    }

    private void sendStateToPub() {
        if (null == sensorID)
            return;
        List<Object> emit = new ArrayList<Object>();
        emit.add(input.getValueByField(Constant_storm.FIELDS.TIME_FIELD));
        emit.add(sensorID);
        emit.add(odometry_);
        emit.add(Footprint_minkowski);
        collector.emit(Constant_storm.Streams.PUBLISHME_STREAM, emit);
    }

    private void snedPrefvel() {
        if (null == sensorID)
            return;
        List<Object> emit = new ArrayList<Object>();
        emit.add(input.getValueByField(Constant_storm.FIELDS.TIME_FIELD));
        emit.add(input.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD));
        emit.add(odometry_);
        emit.add(neighbors);
        emit.add(Footprint_minkowski);
        emit.add(obstacles);
        emit.add(prefVel);
        collector.emit(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM, emit);

    }

    private void updateNeighbors(Map<String, Agent> agents) {
        if (null == sensorID)
            return;
        neighbors.clear();
        for (Map.Entry<String, Agent> e : agents.entrySet()) {
            if (e.getKey().equals(sensorID))
                continue;
            neighbors.add(e.getValue());
        }

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
                Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD,
                Constant_storm.FIELDS.OBSTACLE_FIELD,
                Constant_storm.FIELDS.PREFERRED_VELOCITY_FIELD
        ));
    }
}
