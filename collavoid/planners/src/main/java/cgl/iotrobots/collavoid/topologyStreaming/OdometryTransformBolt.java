package cgl.iotrobots.collavoid.topologyStreaming;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Created by hjh on 1/4/15.
 */
public class OdometryTransformBolt extends BaseBasicBolt {
    private Logger logger = LoggerFactory.getLogger(OdometryTransformBolt.class);

    @Override
    public void execute(Tuple input, BasicOutputCollector collector) {
        Object odoObj = input.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        if (!(odoObj instanceof Odometry_)) {
            logger.error("Received tuple is not an instance of Odometry_!!");
            return;
        }
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

        collector.emit(new Values(input.getValue(0), input.getValue(1), odometry_));
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.ODOMETRY_FIELD
        ));
    }
}
