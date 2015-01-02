package cgl.iotrobots.collavoid.topology;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.PoseShareMsg_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Vector3d_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Created by hjh on 12/24/14.
 */
public class GetAllAgentsBolt extends BaseBasicBolt {
    private Logger logger = LoggerFactory.getLogger(GetAllAgentsBolt.class);
    private Map<String, Agent> Agents = new HashMap<String, Agent>();
    //private List<Agent> Agents = new ArrayList<Agent>();

    @Override
    public void execute(Tuple input, BasicOutputCollector collector) {
        PoseShareMsg_ poseShareMsg_ = (PoseShareMsg_) input.getValueByField(Constant_storm.FIELDS.POSE_SHARE_FIELD);
        updateAgentList(poseShareMsg_);
            List<Object> emit = new ArrayList<Object>();
        emit.add(input.getValueByField(Constant_storm.FIELDS.TIME_FIELD));
        emit.add(Agents);
            collector.emit(emit);
    }


    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.ALL_AGENTS_FIELD));
    }

    private void updateAgentList(PoseShareMsg_ msg) {
        Agent newagent = new Agent(msg.getRobotId());
        newagent.holo_robot_ = msg.getHoloRobot();
        newagent.base_odom_.setPose(msg.getPose());
        newagent.base_odom_.setTwist(msg.getTwist());
        newagent.radius = msg.getRadius();
        newagent.controlled = msg.getControlled();

        List<Vector2> footprint_min = new ArrayList<Vector2>();
        for (Vector3d_ vector3d_ : msg.getFootPrint_Minkowski())
            footprint_min.add(new Vector2(vector3d_.getX(), vector3d_.getY()));

        newagent.footprint_minkowski = footprint_min;
        newagent.last_seen_ = msg.getHeader().getStamp();

        Agents.put(msg.getRobotId(), newagent);
    }
}
