package cgl.iotrobots.collavoid.topology;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.Line;
import cgl.iotrobots.collavoid.commons.planners.Methods_Planners;
import cgl.iotrobots.collavoid.commons.planners.VO;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.util.ArrayList;
import java.util.List;

public class VOAgentBolt extends BaseBasicBolt {
    private Agent agent;
    private int seq = 0;
    private List<VO> voAgents = new ArrayList<VO>();

    @Override
    public void execute(Tuple input, BasicOutputCollector collector) {
        voAgents.clear();
        agent = (Agent) input.getValueByField(Constant_storm.Fields.AGENT_FIELD);
        if (agent.controlled)
            computeAgentVOs();
        collector.emit(new Values(
                input.getValue(0),
                input.getValue(1),
                voAgents,
                seq++));
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.Fields.TIME_FIELD,
                Constant_storm.Fields.SENSOR_ID_FIELD,
                Constant_storm.Fields.VOS_FIELD,
                Constant_storm.Fields.SEQUENCE_FIELD
        ));
    }

    private void computeAgentVOs() {
        double radiusWithError = agent.radius + agent.cur_allowed_error_;
        // neighbors are published with localization uncertainty that means radius and footprint
        // include localization uncertainty radius and minkowski footprint.
        for (Agent neighbor : agent.AgentNeighbors) {
            VO new_agent_vo;
            //use footprint or radius to create VO
            if (agent.convex) {
                if (neighbor.controlled) {
                    new_agent_vo = Methods_Planners.ClearPath.createVO(
                            agent.position.getPos(),
                            agent.footPrint_rotated,
                            agent.velocity,
                            neighbor.position.getPos(),
                            neighbor.footPrint_rotated,
                            neighbor.velocity,
                            agent.voType);
                } else {
                    new_agent_vo = Methods_Planners.ClearPath.createVO(
                            agent.position.getPos(),
                            agent.footPrint_rotated,
                            agent.velocity,
                            neighbor.position.getPos(),
                            neighbor.footPrint_rotated,
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
            }
            voAgents.add(new_agent_vo);
        }
    }
}
