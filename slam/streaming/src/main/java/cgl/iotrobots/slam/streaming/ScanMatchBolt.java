package cgl.iotrobots.slam.streaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.slam.core.gridfastsalm.SharedMemoryGridSlamProcessor;
import cgl.iotrobots.slam.core.sample.GFSAlgorithm;
import cgl.iotrobots.slam.core.sample.LaserScan;
import cgl.iotrobots.slam.core.sensor.RangeReading;

import java.util.Map;

public class ScanMatchBolt extends BaseRichBolt {
    private DistributedScanMatcher gfsp = null;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        gfsp = new DistributedScanMatcher();


    }

    @Override
    public void execute(Tuple tuple) {
        Object val = tuple.getValueByField(Constants.ScanMatchBoltConstants.LASER_SCAN_TUPLE);
        LaserScan laserScan = null;
        if (!(val instanceof LaserScan)) {
            throw new IllegalArgumentException("The laser scan should be of type LaserScan");
        }

        Double[] ranges_double = GFSAlgorithm.getDoubles(laserScan, gfsp.);


        RangeReading reading = new RangeReading(scan.ranges.size(),
                ranges_double,
                gsp_laser_,
                scan.timestamp);
        reading.setPose(gmap_pose);

        gfsp.processScan()
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }
}
