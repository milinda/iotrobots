package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.Obstacle;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.PoseShareMsg_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Vector3d_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;


import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class AgentBolt extends BaseRichBolt {
    private OutputCollector collector;
    private Agent agent;
    private PoseShareMsg_ poseShareMsg_ = new PoseShareMsg_();

    @Override
    public void execute(Tuple tuple) {
        if (agent.Name.equals("")) {
            agent.Name = (String) tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            agent.updateBaseFrame();
        }
        String streamId = tuple.getSourceStreamId();
        if (streamId.equals(Constant_storm.Streams.PUBLISHME_STREAM)) {
            updateAgentToPub(tuple);
            collector.emit(Constant_storm.Streams.PUBLISHME_STREAM, new Values(poseShareMsg_));
        } else if (streamId.equals(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM)) {
            updateAgentToCalVel(tuple);
            collector.emit(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM, new Values(agent));
        }
    }

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
        agent = new Agent();
        collector.emit(Constant_storm.Streams.FOOTPRINT_OWN_STREAM, new Values(agent.footprint_original));
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.PUBLISHME_STREAM,
                new Fields(Constant_storm.FIELDS.POSE_SHARE_FIELD));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                new Fields(Constant_storm.FIELDS.AGENT_FIELD));
    }

    private void updateAgentToPub(Tuple tuple) {
        agent.base_odom_ = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        agent.footprint_minkowski = (List<Vector2>) tuple.getValueByField(Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD);

        poseShareMsg_.getHeader().setFrameId(agent.getBase_frame_());
        poseShareMsg_.getHeader().setStamp(System.currentTimeMillis());
        poseShareMsg_.setPose(agent.getBaseOdom().getPose());
        poseShareMsg_.setTwist(agent.getBaseOdom().getTwist());
        poseShareMsg_.setControlled(agent.getController());
        poseShareMsg_.setHoloRobot(agent.getHoloRobot());
        poseShareMsg_.setRadius(agent.getRadius() + agent.getCur_loc_unc_radius_());
        poseShareMsg_.setRobotId(agent.getName());

        List<Vector3d_> footprint = new ArrayList<Vector3d_>();
        for (Vector2 vector2 : agent.getFootprint_minkowski()) {
            Vector3d_ vector3d_ = new Vector3d_(vector2.getX(), vector2.getY(), 0);
            footprint.add(vector3d_);
        }
        poseShareMsg_.setFootPrint_Minkowski(footprint);
    }

    private void updateAgentToCalVel(Tuple tuple) {
        agent.base_odom_ = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        agent.AgentNeighbors = (List<Agent>) tuple.getValueByField(Constant_storm.FIELDS.NEIGHBORS_FIELD);
        agent.footprint_minkowski = (List<Vector2>) tuple.getValueByField(Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD);
        agent.obstacles_from_laser_ = (List<Obstacle>) tuple.getValueByField(Constant_storm.FIELDS.OBSTACLE_FIELD);
        agent.prefVelociy = (Vector2) tuple.getValueByField(Constant_storm.FIELDS.PREFERRED_VELOCITY_FIELD);
    }
}
