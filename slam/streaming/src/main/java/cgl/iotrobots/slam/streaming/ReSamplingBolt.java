package cgl.iotrobots.slam.streaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.streaming.msgs.ParticleAssignment;
import cgl.iotrobots.slam.streaming.msgs.ParticleAssignments;
import cgl.iotrobots.slam.streaming.msgs.ParticleValues;
import cgl.iotrobots.slam.streaming.rabbitmq.Message;
import cgl.iotrobots.slam.streaming.rabbitmq.RabbitMQSender;
import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.Output;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.ByteArrayOutputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class ReSamplingBolt extends BaseRichBolt {
    private static Logger LOG = LoggerFactory.getLogger(ReSamplingBolt.class);

    private DistributedReSampler reSampler;

    private OutputCollector outputCollector;

    private TopologyContext topologyContext;

    private List<ParticleValues> particleValueses = new ArrayList<ParticleValues>();

    private RangeReading reading;

    private RabbitMQSender sender;

    private BlockingQueue<Message> outQueue = new LinkedBlockingQueue<Message>();

    private String url = "amqp://localhost:";

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        reSampler = new DistributedReSampler();

        this.topologyContext = topologyContext;
        this.outputCollector = outputCollector;
        this.sender = new RabbitMQSender(outQueue, "slam", "particleAssingments", "particle_assignments", url, true);
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

        // this bolt will wait until all the particle values are obtained
        if (particleValueses.size() < reSampler.getNoParticles() || reading == null) {
            return;
        }

        // now distribute the particle Valueses to the bolts

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

        ParticleAssignments assignments = createAssignments(reSampler.getIndexes());
        distributeAssignments(assignments);

        // distribute the new particles

        reading = null;
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }

    /**
     * We will broadcast this message using a topic. Every ScanMatching bolt will receive this message
     * @param assignments the particle assignments
     */
    private void distributeAssignments(ParticleAssignments assignments) {
        Kryo kryo = new Kryo();
        ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
        Output output = new Output();
        kryo.writeObject(output, assignments);
        output.flush();
        byte []b = byteArrayOutputStream.toByteArray();

        Message message = new Message(b, new HashMap<String, Object>());
        try {
            outQueue.put(message);
        } catch (InterruptedException e) {
            LOG.error("Failed to add the message");
        }
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
        int []assignments = algorithm.execute();
        ParticleAssignments particleAssignments = new ParticleAssignments();
        for (int i = 0; i < assignments.length; i++) {
            int thrueTaskIndex = i % noOfParticles;
            int particleIndex = assignments[i];
            int previousParticleIndex = indexes.get(particleIndex);
            ParticleValues pv = particleValueses.get(previousParticleIndex);
            ParticleAssignment assignment = new ParticleAssignment(previousParticleIndex, i,
                    pv.getTaskId(), thrueTaskIndex);
            particleAssignments.addAssignment(assignment);
        }

        return particleAssignments;
    }
}
