package cgl.iotrobots.slam.streaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.streaming.msgs.ParticleValues;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * This bolt is responsible for calculating for the assigned particles,
 * and then distribute the particles to re-sampler
 */
public class ScanMatchBolt extends BaseRichBolt {
    private DistributedScanMatcher gfsp = null;

    private OutputCollector outputCollector;

    private TopologyContext topologyContext;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        gfsp = new DistributedScanMatcher();
        this.outputCollector = outputCollector;
        this.topologyContext = topologyContext;
    }

    @Override
    public void execute(Tuple tuple) {
        Object val = tuple.getValueByField(Constants.ScanMatchBoltConstants.LASER_SCAN_TUPLE);
        RangeReading reading = null;
        if (!(val instanceof RangeReading)) {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }
        int totalTasks = topologyContext.getComponentTasks(topologyContext.getThisComponentId()).size();
        reading = (RangeReading) val;

        gfsp.processScan(reading, 0);

        // now distribute the particles to the bolts
        List<Integer> activeParticles = gfsp.getActiveParticles();
        List<Particle> particles = gfsp.getParticles();

        for (int index : activeParticles) {
            Particle particle = particles.get(index);

            int taskId = topologyContext.getThisTaskIndex();
            ParticleValues particleValues = new ParticleValues(taskId, index, totalTasks, particle.pose,
                    particle.previousPose, particle.weight,
                    particle.weightSum, particle.gweight, particle.previousIndex, particle.node);
            List<Object> emit = new ArrayList<Object>();
            emit.add(particleValues);
            outputCollector.emit(emit);
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new Fields(Constants.ScanMatchBoltConstants.SENSOR_ID_FIELD,
                Constants.ScanMatchBoltConstants.TIME_FIELD,
                Constants.ScanMatchBoltConstants.PARTICLE_VALUE));
    }
}
