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
import cgl.iotrobots.slam.streaming.msgs.ParticleValue;
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

public class ReSamplingBolt extends BaseRichBolt {
    private static Logger LOG = LoggerFactory.getLogger(ReSamplingBolt.class);

    /** We will re sample here */
    private DistributedReSampler reSampler;

    private OutputCollector outputCollector;

    private TopologyContext topologyContext;

    /** we will collect the values until we get all of them */
    private List<ParticleValue> particleValueses = new ArrayList<ParticleValue>();

    /** This is the reading message we will get */
    private RangeReading reading;

    /** The RabbitMQ send used for sending out the re sampled particles back to the scan match bolts */
    private RabbitMQSender assignmentSender;

    private RabbitMQSender valueSender;

    private String url = "amqp://localhost:";

    private Kryo kryo;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        reSampler = new DistributedReSampler();

        this.topologyContext = topologyContext;
        this.outputCollector = outputCollector;
        this.kryo = new Kryo();

        try {
            this.assignmentSender = new RabbitMQSender(url, Constants.Messages.BROADCAST_EXCHANGE);
            this.assignmentSender.open();

            this.valueSender = new RabbitMQSender(url, Constants.Messages.DIRECT_EXCHANGE);
            this.valueSender.open();
        } catch (Exception e) {
            LOG.error("Failed to create the sender", e);
        }
    }

    @Override
    public void execute(Tuple tuple) {
        Object val = tuple.getValueByField(Constants.Fields.PARTICLE_VALUE_FIELD);
        ParticleValue value;
        if (val != null && (val instanceof ParticleValue)) {
            value = (ParticleValue) val;
            particleValueses.add(value.getIndex(), value);
        } else {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }

        val = tuple.getValueByField(Constants.Fields.LASER_SCAN_TUPLE);
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
        for (ParticleValue pv : particleValueses) {
            Particle p = new Particle();
            p.setWeight(pv.getWeight());
            p.setGweight(pv.getGweight());
            p.setPose(pv.getPose());
            p.setNode(pv.getNode());
            p.setPreviousIndex(pv.previousIndex);
            p.setWeightSum(pv.weightSum);

            reSampler.getParticles().add(pv.getIndex(), p);
        }

        // do the resampling
        reSampler.processScan(reading, 0);
        // now distribute the resampled particleValueses
        List<Integer> particles = reSampler.getIndexes();

        // first we will distribute the new assignments
        // this will distribute the current maps
        ParticleAssignments assignments = createAssignments(reSampler.getIndexes());
        distributeAssignments(assignments);

        // distribute the new particle values according to
        for (int i = 0; i < reSampler.getParticles().size(); i++) {
            Particle p = reSampler.getParticles().get(i);
            ParticleValue pv = new ParticleValue(-1, i, -1, p.getPose(), p.getPreviousPose(),
                    p.getWeight(), p.getWeightSum(), p.getGweight(), p.getPreviousIndex(), p.getNode());

            byte []b = Utils.serialize(kryo, pv);
            Message message = new Message(b, new HashMap<String, Object>());
            int taskId = getTaskForParticle(i, assignments.getAssignments());
            try {
                valueSender.send(message, Constants.Messages.PARTICLE_VALUE_ROUTING_KEY + "_" + taskId);
            } catch (Exception e) {
                LOG.error("Failed to send the message");
            }
        }

        reading = null;
    }

    private int getTaskForParticle(int particleIndex, List<ParticleAssignment> assignments) {
        return 0;
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }

    /**
     * We will broadcast this message using a topic. Every ScanMatching bolt will receive this message
     * @param assignments the particle assignments
     */
    private void distributeAssignments(ParticleAssignments assignments) {
        byte []b = Utils.serialize(kryo, assignments);

        Message message = new Message(b, new HashMap<String, Object>());
        try {
            assignmentSender.send(message, "all");
        } catch (Exception e) {
            LOG.error("Failed to send the message", e);
        }
    }

    /**
     * This method create an assignment of the resampled particles to the tasks running in Storm.
     * In this case the tasks wille.printStackTrace(); be the bolts running the ScanMatching code.
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
                ParticleValue pv = particleValueses.get(index);
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

        // go through the particle indexs and try to find their new assignments
        for (int i = 0; i < indexes.size(); i++) {
            int particle = indexes.get(i);
            int thrueTaskIndex = -1;
            for (int j = 0; j < assignments.length; j++) {
                if (assignments[j] == i) {
                    thrueTaskIndex = j % noOfParticles;
                }
            }
            ParticleValue pv = particleValueses.get(particle);
            ParticleAssignment assignment = new ParticleAssignment(particle, i,
                    pv.getTaskId(), thrueTaskIndex);
            particleAssignments.addAssignment(assignment);
        }

        return particleAssignments;
    }
}
