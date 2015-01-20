package cgl.iotrobots.collavoid.topologyStreaming;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.Line;
import cgl.iotrobots.collavoid.commons.planners.Methods_Planners;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by hjh on 12/25/14.
 */
public class AddAccelerationConstraintBolt extends BaseBasicBolt {
    private Agent agent;
    private int seq = 0;
    private Logger logger = LoggerFactory.getLogger(AddAccelerationConstraintBolt.class);

    @Override
    public void execute(Tuple input, BasicOutputCollector collector) {
        agent = (Agent) Utils.deserialize(input.getBinaryByField(Constant_storm.FIELDS.AGENT_FIELD));
        List<Line> accConstLines = new ArrayList<Line>();
        Methods_Planners.NHORCA.addAccelerationConstraintsXY(
                agent.max_vel_x_,
                agent.acc_lim_x_,
                agent.max_vel_y_,
                agent.acc_lim_y_,
                agent.velocity,
                agent.position.getHeading(),
                agent.controlPeriod,
                agent.holo_robot_,
                accConstLines
        );
        collector.emit(new Values(
                        input.getValue(0),
                        input.getValue(1),
                        Utils.serialize(agent),
                        accConstLines,
                        seq++)
        );

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
