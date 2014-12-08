package cgl.iotrobots.slam.streaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.streaming.msgs.ParticleAssignments;
import cgl.iotrobots.slam.streaming.msgs.ParticleValues;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class ReSamplingBolt extends BaseRichBolt {
    private DistributedReSampler reSampler;

    private OutputCollector outputCollector;

    private TopologyContext topologyContext;

    private List<ParticleValues> particleValueses = new ArrayList<ParticleValues>();

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
            particleValueses.add(value.getIndex(), value);
        } else {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }

        val = tuple.getValueByField(Constants.ScanMatchBoltConstants.LASER_SCAN_TUPLE);
        if (val != null && !(val instanceof RangeReading)) {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }
        reading = (RangeReading) val;

        // now distribute the particleValueses to the bolts
        if (particleValueses.size() < reSampler.getNoParticles() || reading == null) {
            return;
        }

        // we got all the particleValueses, we will resample
        // first we need to clear the current particleValueses
        reSampler.getParticles().clear();
        for (ParticleValues pv : particleValueses) {
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
        // now distribute the resampled particleValueses
        List<Integer> particles = reSampler.getIndexes();


        reading = null;
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }

    /**
     * This method create an assignment of the resampled particles to the tasks running in Storm.
     * In this case the tasks will be the bolts running the ScanMatching code.
     * @param indexes the re sampled indexes
     * @return an assignment of particles
     */
    private ParticleAssignments createAssignments(List<Integer> indexes) {
        // create a matrix of size noOfParticles x noOfparticles
        int noOfParticles = reSampler.getNoParticles();

        // assume taskIndexes are going from 0
        double [][]cost = new double[noOfParticles][noOfParticles];
        for (int i = 0; i < noOfParticles; i++) {
            cost[i] = new double[noOfParticles];
            for (int j = 0; j < noOfParticles; j++) {
                int index = indexes.get(j);
                ParticleValues pv = particleValueses.get(index);
                // now see weather this particle is from this worker
                int particleTaskIndex = pv.getTaskId();
                int thrueTaskIndex = j % noOfParticles;
                if (particleTaskIndex == thrueTaskIndex) {
                    cost[i][j] = 1;
                } else {
                    cost[i][j] = 0;
                }
            }
        }

        HungarianAlgorithm algorithm = new HungarianAlgorithm(cost);
        int []assingments = algorithm.execute();

        return null;
    }
}
