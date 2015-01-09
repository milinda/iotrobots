package cgl.iotrobots.collavoid.topologyStreaming;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.planners.Neighbor;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.PoseShareMsg_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Created by hjh on 12/24/14.
 */
public class GetAllAgentsBolt extends BaseBasicBolt {
    private Logger logger = LoggerFactory.getLogger(GetAllAgentsBolt.class);
    private Map<String, Neighbor> Agents = new HashMap<String, Neighbor>();
    private PoseShareMsg_ poseShareMsg_;
    //private List<Agent> Agents = new ArrayList<Agent>();

    @Override
    public void execute(Tuple input, BasicOutputCollector collector) {
        if (input.getSourceStreamId().equals(Constant_storm.Streams.RESET_STREAM)) {
            // here we simply clear all agents but in more practical situation this is
            // not good.
            String sensorID = input.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            if (Agents.containsKey(sensorID)) {
                Agents.remove(sensorID);
            }
        } else {
            poseShareMsg_ = (PoseShareMsg_) input.getValueByField(Constant_storm.FIELDS.POSE_SHARE_FIELD);
            updateAgentList(poseShareMsg_);
        }
        List<Object> emit = new ArrayList<Object>();
        emit.add(input.getLongByField(Constant_storm.FIELDS.TIME_FIELD));
        emit.add(Utils.serialize(Agents));
        collector.emit(emit);
    }


    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.ALL_AGENTS_FIELD));
    }

    private void updateAgentList(PoseShareMsg_ msg) {
        Neighbor newagent = new Neighbor(msg.getName());
        newagent.holo_robot_ = msg.getHoloRobot();
        newagent.base_odom_ = new Odometry_();
        newagent.base_odom_.setPose(msg.getPose());
        newagent.base_odom_.setTwist(msg.getTwist());
        newagent.radius = msg.getRadius();
        newagent.controlled = msg.getControlled();
        newagent.setControlPeriod(msg.getControlPeriod());
        newagent.setFootprint_minkowski(msg.getFootPrint_Minkowski());
        newagent.last_seen_ = msg.getHeader().getStamp();

        Agents.put(msg.getName(), newagent);
    }
}
