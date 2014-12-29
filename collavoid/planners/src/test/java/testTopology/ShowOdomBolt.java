package testTopology;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;

public class ShowOdomBolt extends BaseBasicBolt {
    @Override
    public void execute(Tuple tuple, BasicOutputCollector basicOutputCollector) {
        Odometry_ odometry_ = (Odometry_) tuple.getValueByField(Constant.FIELDS.ODOMETRY_FIELD);
        System.out.println(odometry_.getHeader().getStamp());
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }
}
