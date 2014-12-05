package cgl.iotrobots.slam.streaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.streaming.msgs.ParticleValues;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class ReSamplingBolt extends BaseRichBolt {
    private DistributedReSampler reSampler;

    private OutputCollector outputCollector;

    private TopologyContext topologyContext;

    private List<Particle> particles = new ArrayList<Particle>();

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        reSampler = new DistributedReSampler();

        this.topologyContext = topologyContext;
        this.outputCollector = outputCollector;
    }

    @Override
    public void execute(Tuple tuple) {
        Object val = tuple.getValueByField(Constants.ScanMatchBoltConstants.LASER_SCAN_TUPLE);
        RangeReading reading = null;
        if (!(val instanceof RangeReading)) {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }

        reading = (RangeReading) val;

        reSampler.processScan(reading, 0);

        // now distribute the particles to the bolts
        List<Particle> particles = reSampler.getParticles();

        for (Particle particle : particles) {
            int taskId = topologyContext.getThisTaskIndex();
            List<Object> emit = new ArrayList<Object>();
            emit.add();
            outputCollector.emit(emit);
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }
}
