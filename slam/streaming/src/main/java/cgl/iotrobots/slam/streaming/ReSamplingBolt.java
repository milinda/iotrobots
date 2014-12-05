package cgl.iotrobots.slam.streaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.streaming.msgs.ParticleValues;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class ReSamplingBolt extends BaseRichBolt {
    private DistributedReSampler reSampler;

    private OutputCollector outputCollector;

    private TopologyContext topologyContext;

    private List<ParticleValues> particles = new ArrayList<ParticleValues>();

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
        if (!(val instanceof ParticleValues)) {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }
        value = (ParticleValues) val;

        particles.add(value.getIndex(), value);
        // now distribute the particles to the bolts
        if (particles.size() < reSampler.getNoParticles()) {
            return;
        }

        // we got all the particles, we will resample
        reSampler.getParticles().clear();
        for (ParticleValues values : particles) {
            Particle p = pa

            reSampler.getParticles().add(va)
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }
}
