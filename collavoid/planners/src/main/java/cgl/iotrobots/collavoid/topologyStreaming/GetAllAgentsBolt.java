package cgl.iotrobots.collavoid.topologyStreaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.TimeDelayAnalysis.Constants;
import cgl.iotrobots.collavoid.commons.TimeDelayAnalysis.TimeDelayRecorder;
import cgl.iotrobots.collavoid.commons.planners.Neighbor;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.PoseShareMsg_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import com.esotericsoftware.kryo.Kryo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Created by hjh on 12/24/14.
 */
public class GetAllAgentsBolt extends BaseRichBolt {
    private Logger logger = LoggerFactory.getLogger(GetAllAgentsBolt.class);
    private Map<String, Neighbor> Agents = new HashMap<String, Neighbor>();
    private PoseShareMsg_ poseShareMsg_;
    private OutputCollector collector;
    private Kryo kryo;
    TimeDelayRecorder poseShareDelayRecorder;
    //private List<Agent> Agents = new ArrayList<Agent>();

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
        kryo = Methods_RMQ.getKryo();
        poseShareDelayRecorder = new TimeDelayRecorder(
                Constants.PARAMETER_DELAY,
                Constant_msg.KEY_POSE_SHARE,
                topologyContext.getThisComponentId());
        poseShareDelayRecorder.open(false);
    }

    @Override
    public void execute(Tuple input) {
        if (input.getSourceStreamId().equals(Constant_storm.Streams.RESET_STREAM)) {
            // here we simply clear all agents but in more practical situation this is
            // not good.
            String sensorID = input.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            if (Agents.containsKey(sensorID)) {
                Agents.remove(sensorID);
            }
        } else {
            poseShareDelayRecorder.append(
                    input.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                    input.getLongByField(Constant_storm.FIELDS.TIME_FIELD),
                    System.currentTimeMillis());
            poseShareMsg_ = (PoseShareMsg_) Methods_RMQ.deSerialize(
                    kryo,
                    (byte[]) input.getValueByField(Constant_storm.FIELDS.POSE_SHARE_FIELD)
            );
            updateAgentList(poseShareMsg_);
        }
        collector.emit(new Values(
                input.getLongByField(Constant_storm.FIELDS.TIME_FIELD),
                input.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                Utils.serialize(Agents)
        ));
        collector.ack(input);
    }


    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
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
