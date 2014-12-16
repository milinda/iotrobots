package cgl.iotrobots.slam.streaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.slam.core.GFSConfiguration;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.sensor.RangeSensor;
import cgl.iotrobots.slam.core.sensor.Sensor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.streaming.msgs.ParticleAssignment;
import cgl.iotrobots.slam.streaming.msgs.ParticleAssignments;
import cgl.iotrobots.slam.streaming.msgs.ParticleValue;
import cgl.iotrobots.utils.rabbitmq.Message;
import cgl.iotrobots.utils.rabbitmq.RabbitMQSender;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import com.esotericsoftware.kryo.Kryo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

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
    private ParticleValue []particleValueses;

    /** This is the reading message we will get */
    private RangeReading reading;

    /** The RabbitMQ send used for sending out the re sampled particles back to the scan match bolts */
    private RabbitMQSender assignmentSender;

    private RabbitMQSender valueSender;

    private String url = "amqp://localhost:5672";

    private Kryo kryo;

    private int receivedParticles = 0;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        this.topologyContext = topologyContext;
        this.outputCollector = outputCollector;
        this.kryo = new Kryo();
        // read the configuration of the scanmatcher from topology.xml
        StreamTopologyBuilder streamTopologyBuilder = new StreamTopologyBuilder();
        StreamComponents components = streamTopologyBuilder.buildComponents();
        // use the configuration to create the resampler
        GFSConfiguration cfg = ConfigurationBuilder.getConfiguration(components.getConf());
        reSampler = ProcessorFactory.createReSampler(cfg);
        particleValueses = new ParticleValue[reSampler.getParticles().size()];
        this.url = (String) components.getConf().get(Constants.RABBITMQ_URL);
        try {
            this.assignmentSender = new RabbitMQSender(url, Constants.Messages.BROADCAST_EXCHANGE, true);
            this.assignmentSender.open();

            this.valueSender = new RabbitMQSender(url, Constants.Messages.DIRECT_EXCHANGE);
            this.valueSender.open();
        } catch (Exception e) {
            LOG.error("Failed to create the sender", e);
        }
    }

    @Override
    public void execute(Tuple tuple) {
        outputCollector.ack(tuple);

        Object val = tuple.getValueByField(Constants.Fields.PARTICLE_VALUE);
        ParticleValue value;

        if (val != null && (val instanceof ParticleValue)) {
            value = (ParticleValue) val;
            LOG.info("Received particle with index {}", value.getIndex());
            particleValueses[value.getIndex()] = value;
            receivedParticles++;
        } else {
            throw new IllegalArgumentException("The particle value should be of type ParticleValue");
        }

        val = tuple.getValueByField(Constants.Fields.LASER_SCAN_TUPLE);
        if (val != null && !(val instanceof LaserScan)) {
            throw new IllegalArgumentException("The laser scan should be of type LaserScan");
        }
        LaserScan scan = (LaserScan) val;
        Double[] ranges_double = cgl.iotrobots.slam.core.utils.Utils.getDoubles(scan, scan.getAngle_increment());
        RangeSensor sensor = new RangeSensor("ROBOTLASER1",
                scan.getRanges().size(),
                Math.abs(scan.getAngle_increment()),
                new DoubleOrientedPoint(0, 0, 0),
                0.0,
                scan.getRangeMax());

        reading = new RangeReading(scan.getRanges().size(),
                ranges_double,
                sensor,
                scan.getTimestamp());
        reading.setPose(scan.getPose());
        Map<String, Sensor> smap = new HashMap<String, Sensor>();
        smap.put(sensor.getName(), sensor);
        reSampler.setSensorMap(smap);

        LOG.info("receivedParticles {}", receivedParticles);
        // this bolt will wait until all the particle values are obtained
        if (receivedParticles < reSampler.getNoParticles() || reading == null) {
            return;
        }
        // reset the counter
        receivedParticles = 0;

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
        boolean hasReSampled = reSampler.processScan(reading, 0);
        // now distribute the resampled particleValueses
        List<Integer> particles = reSampler.getIndexes();

        // we will distribute only if we have reSampled
        if (hasReSampled) {
            // first we will distribute the new assignments
            // this will distribute the current maps
            ParticleAssignments assignments = createAssignments(reSampler.getIndexes());
            assignments.setReSampled(hasReSampled);
            distributeAssignments(assignments);

            // distribute the new particle values according to
            for (int i = 0; i < reSampler.getParticles().size(); i++) {
                Particle p = reSampler.getParticles().get(i);
                ParticleValue pv = new ParticleValue(-1, i, -1, p.getPose(), p.getPreviousPose(),
                        p.getWeight(), p.getWeightSum(), p.getGweight(), p.getPreviousIndex(), p.getNode());

                byte[] b = Utils.serialize(kryo, pv);
                Message message = new Message(b, new HashMap<String, Object>());
                // we assume there is a direct mapping between particles in the resampler and the indexes
                ParticleAssignment assignment = assignments.getAssignments().get(i);
                try {
                    valueSender.send(message, Constants.Messages.PARTICLE_VALUE_ROUTING_KEY + "_" + assignment.getNewTask());
                } catch (Exception e) {
                    LOG.error("Failed to send the message");
                }
            }
        } else {
            ParticleAssignments assignments = new ParticleAssignments();
            assignments.setReSampled(false);
            distributeAssignments(assignments);
        }

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
                ParticleValue pv = particleValueses[index];
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
            ParticleValue pv = particleValueses[particle];
            ParticleAssignment assignment = new ParticleAssignment(particle, i,
                    pv.getTaskId(), thrueTaskIndex);
            particleAssignments.addAssignment(assignment);
        }

        return particleAssignments;
    }
}
