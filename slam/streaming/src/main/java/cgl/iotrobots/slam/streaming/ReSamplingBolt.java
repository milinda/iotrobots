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

    private List<ParticleValues> particles = new ArrayList<ParticleValues>();

    private RangeReading reading;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        reSampler = new DistributedReSampler();

        this.topologyContext = topologyContext;
        this.outputCollector = outputCollector;
    }

    @Override
    public void execute(Tuple tuple) {
        Object val = tuple.getValueByField(Constants.ScanMatchBoltConstants.PARTICLE_VALUE_FIELD);
        ParticleValues value;
        if (val != null && (val instanceof ParticleValues)) {
            value = (ParticleValues) val;
            particles.add(value.getIndex(), value);
        } else {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }

        val = tuple.getValueByField(Constants.ScanMatchBoltConstants.LASER_SCAN_TUPLE);
        if (val != null && !(val instanceof RangeReading)) {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }
        reading = (RangeReading) val;

        // now distribute the particles to the bolts
        if (particles.size() < reSampler.getNoParticles() || reading == null) {
            return;
        }

        // we got all the particles, we will resample
        // first we need to clear the current particles
        reSampler.getParticles().clear();
        for (ParticleValues pv : particles) {
            Particle p = new Particle();
            p.setWeight(pv.getWeight());
            p.setGweight(pv.getGweight());
            p.setPose(pv.getPose());
            p.setNode(pv.getNode());
            p.setPreviousIndex(pv.previousIndex);
            p.setWeightSum(pv.weightSum);

            reSampler.getParticles().add(pv.getIndex(), p);
        }

        reSampler.processScan(reading, 0);

        // now distribute the resampled particles

        reading = null;
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }
}
