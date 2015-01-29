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
import cgl.iotrobots.collavoid.commons.TimeDelayAnalysis.Constants;
import cgl.iotrobots.collavoid.commons.TimeDelayAnalysis.TimeDelayRecorder;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Map;

/**
 * Created by hjh on 1/4/15.
 */
public class OdometryTransformBolt extends BaseRichBolt {
    private Logger logger = LoggerFactory.getLogger(OdometryTransformBolt.class);
    private TimeDelayRecorder delayRecorder;
    private OutputCollector collector;

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.ODOMETRY_FIELD
        ));
    }

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
        delayRecorder = new TimeDelayRecorder(Constants.PARAMETER_DELAY,
                Constant_msg.KEY_ODOMETRY,
                topologyContext.getThisComponentId());
        delayRecorder.open(false);
    }

    @Override
    public void execute(Tuple tuple) {
        Object odoObj = tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        if (!(odoObj instanceof Odometry_)) {
            logger.error("Received tuple is not an instance of Odometry_!!");
            collector.ack(tuple);
            return;
        }
        delayRecorder.append(
                tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                tuple.getLongByField(Constant_storm.FIELDS.TIME_FIELD), 
                System.currentTimeMillis());

        Odometry_ odometry_ = (Odometry_) odoObj;
        odometry_ = odometry_.copy();

/*      In the original program, odometry is published in robot frame, but it need velocities in
        robot frame, and pose in global frame. So it directly cast the twist to base odom but
        transformed the pose into global frame.
        In our context both velocity and position are in odometry frame so only need to transform
        velocity to robot frame. Currently map frame and odometry frame are the same.*/

        double vx = odometry_.getTwist().getLinear().getX();
        double vy = odometry_.getTwist().getLinear().getY();
        double vxTransformed = Math.sqrt(vx * vx + vy * vy);
        odometry_.getTwist().getLinear().setX(vxTransformed);
        odometry_.getTwist().getLinear().setY(0.0);

        collector.emit(new Values(tuple.getValue(0), tuple.getValue(1), odometry_));
        collector.ack(tuple);
    }
}
