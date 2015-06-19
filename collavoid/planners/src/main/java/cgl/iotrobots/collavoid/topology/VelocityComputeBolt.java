package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.AgentState;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import com.esotericsoftware.kryo.Kryo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class VelocityComputeBolt extends BaseRichBolt {
    Logger logger = LoggerFactory.getLogger(VelocityComputeBolt.class);
    Kryo kryo;
    OutputCollector collector;
    Map<String, Agent> agentMap = new ConcurrentHashMap<String, Agent>();
    long arrTime;
    long emitTime;
    int taskid;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
        kryo = Methods_RMQ.getKryo();
        taskid = topologyContext.getThisTaskId();
    }

    @Override
    public void execute(Tuple tuple) {
        arrTime = System.currentTimeMillis();
        if (tuple.getSourceComponent().equals(Constant_storm.Components.AGENT_STATE_COMPONENT)) {
            emitNewCmd(tuple);
        } else if (tuple.getSourceComponent().equals(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT)) {
            cacheAgent(tuple);
        }
        collector.ack(tuple);
    }

    private void cacheAgent(Tuple tuple) {
        Agent agent = (Agent) tuple.getValueByField(Constant_storm.FIELDS.AGENT_FIELD);
        agentMap.put(agent.id, agent);
        collector.emit(Constant_storm.Streams.ACK_STREAM, new Values(
                agent.id,
                agent.getSeq(),
                tuple.getValueByField(Constant_storm.FIELDS.AGENT_IDX_FIELD)
        ));
    }

    private void emitNewCmd(Tuple tuple) {
        boolean goalReached = false;
        Object input = tuple.getValueByField(Constant_storm.FIELDS.AGENT_STATE_FIELD);
        String times = tuple.getStringByField(Constant_storm.FIELDS.EMIT_TIME_FIELD);
        times = times + arrTime + ",";
        if (input != null) {
            AgentState agentState = (AgentState) input;
            Agent agent = agentMap.get(agentState.id);
            int flag = updateAgentState(agentState, agent);
            if (flag == 0) {
                Methods.getCommand(agent);
                if (agent.cmd_vel.isGoalReached()) {
                    agentMap.remove(agentState.id);
                    goalReached = true;
                }
                collector.emit(Constant_storm.Streams.VELOCITY_COMMAND_STREAM, new Values(
                        tuple.getValue(0),
                        tuple.getValue(1),
                        Methods_RMQ.serialize(kryo, agent.cmd_vel)
                ));
            } else if (flag < 0) {
            } else {
            }
        }
        emitTime = System.currentTimeMillis();
        collector.emit(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM, new Values(
                tuple.getValue(0),
                tuple.getValue(1),
                goalReached,
                times + emitTime + ",",
                taskid,
                tuple.getValueByField(Constant_storm.FIELDS.AGENT_IDX_FIELD)
        ));
    }

    private int updateAgentState(AgentState agentState, Agent agent) {
        // agent is not received from global planner or odometry is old
        if (null == agent || agent.getSeq() > agentState.odometry_.getHeader().getSeq()) {
            return 1;
        } else if (agent.getSeq() == agentState.odometry_.getHeader().getSeq()) {
            agent.setBase_odom_(agentState.odometry_);
            agent.setAgentNeighbors(agentState.neighbors);
            agent.setObstacles_from_laser_(agentState.obstacles);
            agent.setFootprint_minkowski(agentState.minkowskiFootprint);
            return 0;
        } else {
            return -1;
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.VELOCITY_COMMAND_STREAM, new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.VELOCITY_COMMAND_FIELD
        ));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM, new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.AGENT_STATE_FIELD,
                Constant_storm.FIELDS.EMIT_TIME_FIELD,
                Constant_storm.FIELDS.TASK_ID_FIELD,
                Constant_storm.FIELDS.AGENT_IDX_FIELD
        ));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.ACK_STREAM, new Fields(
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.SEQUENCE_FIELD,
                Constant_storm.FIELDS.AGENT_IDX_FIELD
        ));
    }
}
