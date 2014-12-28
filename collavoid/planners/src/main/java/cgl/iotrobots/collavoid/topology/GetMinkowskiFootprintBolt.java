package cgl.iotrobots.collavoid.topology;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.planners.Methods_Planners;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.PoseArray_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.util.ArrayList;
import java.util.List;

public class GetMinkowskiFootprintBolt extends BaseBasicBolt {
    private List<Vector2> MinkowskiFootprint = new ArrayList<Vector2>();
    private List<Vector2> ownFootprint = new ArrayList<Vector2>();

    @Override
    public void execute(Tuple input, BasicOutputCollector collector) {
        if (input.getSourceComponent().equals(Constant_storm.Components.PARTICLE_CLOUD_COMPONENT)
                && ownFootprint.size() > 0) {
            PoseArray_ poseArray_ = (PoseArray_) input.getValueByField(Constant_storm.FIELDS.POSE_ARRAY_FIELD);
            getMinkowskiFootprint(poseArray_);
            List<Object> emit = new ArrayList<Object>();
            emit.add(input.getValue(0));
            emit.add(input.getValue(1));
            emit.add(MinkowskiFootprint);
            collector.emit(emit);
        } else if (input.getSourceStreamId().equals(Constant_storm.Streams.FOOTPRINT_OWN_STREAM)) {
            ownFootprint = (List<Vector2>) input.getValueByField(Constant_storm.FIELDS.FOOTPRINT_OWN_FIELD);
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD
        ));
    }

    //for test replaced the algorithm computeNewMinkowskiFootprint
    private void getMinkowskiFootprint(PoseArray_ poseArray_) {
        // in robot base frame do not need transform
        double x, y;
        List<Vector2> localization_footprint = new ArrayList<Vector2>();
        // select valid localization
        for (int i = 0; i < poseArray_.getPoses().size(); i++) {
            x = poseArray_.getPoses().get(i).getPosition().getX();
            y = poseArray_.getPoses().get(i).getPosition().getY();
            Vector2 p = new Vector2(x, y);
            if (p.getLength() > 0.1)
                continue;
            localization_footprint.add(p);
        }
        MinkowskiFootprint = Methods_Planners.minkowskiSumConvexHull(localization_footprint, ownFootprint);
    }
}
