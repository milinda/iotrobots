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
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by hjh on 12/25/14.
 */
public class AddAccelerationConstraintBolt extends BaseBasicBolt {
    private Agent agent;
    private int seq = 0;
    private List<Line> accConstLines = new ArrayList<Line>();

    @Override
    public void execute(Tuple input, BasicOutputCollector collector) {
        agent = (Agent) input.getValueByField(Constant_storm.FIELDS.AGENT_FIELD);
        Methods_Planners.NHORCA.addAccelerationConstraintsXY(
                agent.max_vel_x_,
                agent.acc_lim_x_,
                agent.max_vel_y_,
                agent.acc_lim_y_,
                agent.velocity,
                agent.position.getHeading(),
                agent.ControlPeriod,
                agent.holo_robot_,
                accConstLines
        );
        collector.emit(new Values(input.getValue(0), input.getValue(1), agent, accConstLines, seq++));
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.AGENT_FIELD,
                Constant_storm.FIELDS.ACC_LINES_FIELD,
                Constant_storm.FIELDS.SEQUENCE_FIELD));
    }
}
