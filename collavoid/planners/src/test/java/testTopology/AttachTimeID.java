package testTopology;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.StartGoal_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import com.rabbitmq.client.AMQP;

public class AttachTimeID extends BaseBasicBolt {
    private String sensorID;
    private String bodyField;

    public AttachTimeID(String id, String field) {
        sensorID = id;
        bodyField = field;
    }
    @Override
    public void execute(Tuple tuple, BasicOutputCollector basicOutputCollector) {
//        if (tuple.getSourceComponent().equals(Constant_storm.Components.START_GOAL_COMPONENT+"Spout")) {
//            StartGoal_ startGoal_=(StartGoal_)tuple.getValue(0);
//            System.out.println("bolt "+sensorID + ":  start " +startGoal_.getStart().getPose().toString()
//                    +"; goal "+startGoal_.getGoal().getPose().toString() );
//        }
        basicOutputCollector.emit(new Values(System.currentTimeMillis(), sensorID, tuple.getValue(0)));
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                bodyField
        ));
    }
}
