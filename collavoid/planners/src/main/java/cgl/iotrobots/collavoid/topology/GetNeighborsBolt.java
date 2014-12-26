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
import java.util.List;
import java.util.logging.Logger;

/**
 * Created by hjh on 12/24/14.
 */
public class GetNeighborsBolt extends BaseBasicBolt {
    private Logger logger = Logger.getLogger("GetNeighborsBolt");
    private String robotID = "";// use sensor id as robot id
    private List<Agent> neighbors = new ArrayList<Agent>();

    @Override
    public void execute(Tuple input, BasicOutputCollector collector) {
        if (input.getSourceComponent().equals(Constant_storm.Components.POSE_SHARE_COMPONENT) &&
                !robotID.equals("")) {
            PoseShareMsg_ poseShareMsg_ = (PoseShareMsg_) input.getValueByField(Constant_storm.Fields.POSE_SHARE_FIELD);
            getNeighbors(poseShareMsg_);
            List<Object> emit = new ArrayList<Object>();
            emit.add(neighbors);
            collector.emit(emit);
        } else if (input.getSourceComponent().equals(Constant_storm.Components.AGENT_COMPONENT)) {
            robotID = (String) input.getValueByField(Constant_storm.Fields.SENSOR_ID_FIELD);
        }
    }


    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(Constant_storm.Fields.NEIGHBORS_FIELD));
    }

    private void getNeighbors(PoseShareMsg_ msg) {
        String cur_id = msg.getRobotId();
        if (!cur_id.equals(robotID)) {  //if it is not me do something
            int i;
            for (i = 0; i < neighbors.size(); i++) {
                if (neighbors.get(i).getRobotId().equals(cur_id)) {
                    //I found the robot
                    break;
                }
            }
            if (i >= neighbors.size()) { //Robot is new, so it will be added to the list
                Agent new_robot = new Agent(cur_id);
                new_robot.setHolo_robot_(msg.getHoloRobot());
                neighbors.add(new_robot);
                logger.info(robotID + " added a new neighbor with AgentName " +
                        cur_id + " and radius " + msg.getRadius());
            }

            Agent lstagent = neighbors.get(i);
            lstagent.getBaseOdom().setPose(msg.getPose());
            lstagent.getBaseOdom().setTwist(msg.getTwist());

            lstagent.setRadius(msg.getRadius());
            lstagent.setControlled(msg.getControlled());
            List<Vector2> footprint = new ArrayList<Vector2>();
            for (Vector3d_ vector3d_ : msg.getFootPrint_Minkowski())
                footprint.add(new Vector2(vector3d_.getX(), vector3d_.getY()));
            lstagent.setFootprint_minkowski(footprint);
            lstagent.setLast_seen_(msg.getHeader().getStamp());
        }

    }
}
