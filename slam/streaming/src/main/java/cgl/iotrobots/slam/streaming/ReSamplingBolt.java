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
import java.util.regex.Pattern;

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
        Utils.registerClasses(kryo);
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

    long lastMessageTime;
    long lastComputationTime;

    @Override
    public void execute(Tuple tuple) {
        outputCollector.ack(tuple);

        Object val = tuple.getValueByField(Constants.Fields.PARTICLE_VALUE_FIELD);
        List<ParticleValue> pvs;
        Object time = tuple.getValueByField(Constants.Fields.TIME_FIELD);
        long t0 = System.currentTimeMillis();
        long currentMessageTime = Long.parseLong(time.toString());
        // if this message came within that window, discard it
        // this will allow us to keep track of the current interval
        if (currentMessageTime < lastComputationTime * 4 + lastMessageTime) {
            outputCollector.ack(tuple);
            return;
        }

        if (val != null && (val instanceof List)) {
            pvs = (List<ParticleValue>) val;
            for (ParticleValue value : pvs) {
                LOG.debug("Received particle with index {}", value.getIndex());
                addParticleValue(value);
            }
        } else {
            throw new IllegalArgumentException("The particle value should be of type ParticleValue");
        }

        val = tuple.getValueByField(Constants.Fields.LASER_SCAN_FIELD);
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

        LOG.info("receivedParticles: {}, expecting particles:{}", receivedParticles, reSampler.getParticles().size());
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
            Utils.createParticle(pv, p);

            reSampler.getParticles().add(pv.getIndex(), p);
        }

        // do the resampling
        boolean hasReSampled = reSampler.processScan(reading, 0);
        // now distribute the resampled particleValueses
        List<Integer> particles = reSampler.getIndexes();

        int best = reSampler.getBestParticleIndex();
        // we will distribute only if we have reSampled
        if (hasReSampled) {
            // first we will distribute the new assignments
            // this will distribute the current maps
            LOG.info("ReSampled, distributing assignments");
            ParticleAssignments assignments = createAssignments(reSampler.getIndexes());
            assignments.setReSampled(true);
            assignments.setBestParticle(best);
            distributeAssignments(assignments);

            // distribute the new particle values according to
            for (int i = 0; i < reSampler.getParticles().size(); i++) {
                Particle p = reSampler.getParticles().get(i);
//                ParticleValue pv = new ParticleValue(-1, i, -1, p.getPose(), p.getPreviousPose(),
//                        p.getWeight(), p.getWeightSum(), p.getGweight(), p.getPreviousIndex(), p.getNode());
                ParticleValue pv = Utils.createParticleValue(p, -1, i, -1);

                byte[] b = Utils.serialize(kryo, pv);
                Message message = new Message(b, new HashMap<String, Object>());
                // we assume there is a direct mapping between particles in the resampler and the indexes
                ParticleAssignment assignment = assignments.getAssignments().get(i);
                try {
                    if (i == best) {
                        LOG.info("Best node index: {}, sending this to task: {}", i, assignment.getNewTask());
                        pv.setBest(true);
                    }

                    LOG.info("Sending particle value to: {}", assignment.getNewTask());
                    valueSender.send(message, Constants.Messages.PARTICLE_VALUE_ROUTING_KEY + "_" + assignment.getNewTask());
                } catch (Exception e) {
                    LOG.error("Failed to send the message");
                }
            }
        } else {
            LOG.info("NOT ReSampled, distributing assignments");
            ParticleAssignments assignments = new ParticleAssignments();
            assignments.setReSampled(false);
            assignments.setBestParticle(best);
            distributeAssignments(assignments);
        }
        lastComputationTime = System.currentTimeMillis() - t0;
        lastMessageTime = Long.parseLong(time.toString());

        reading = null;
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }

    protected void addParticleValue(ParticleValue value) {
        particleValueses[value.getIndex()] = value;
        receivedParticles++;
    }

    /**
     * We will broadcast this message using a topic. Every ScanMatching bolt will receive this message
     * @param assignments the particle assignments
     */
    private void distributeAssignments(ParticleAssignments assignments) {
        LOG.info("Sending particle assignment");
        byte []b = Utils.serialize(kryo, assignments);

        Message message = new Message(b, new HashMap<String, Object>());
        try {
            assignmentSender.send(message, "*");
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
    protected ParticleAssignments createAssignments(List<Integer> indexes) {
        for (int i : indexes) {
            System.out.format("%d ", i);
        }
        System.out.format("\n");
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
                int thrueTaskIndex = i % pv.getTotalTasks();
                if (particleTaskIndex == thrueTaskIndex) {
                    cost[i][j] = 0;
                } else {
                    cost[i][j] = 1;
                }
            }
        }

        for (int i = 0; i < cost.length; i++) {
            for (int j = 0; j < cost[i].length; j++) {
                System.out.format("%f ", cost[i][j]);
            }
            System.out.format("\n");
        }

        HungarianAlgorithm algorithm = new HungarianAlgorithm(cost);
        int []assignments = algorithm.execute();
        ParticleAssignments particleAssignments = new ParticleAssignments();

        for (int i : assignments) {
            System.out.format("%d ", i);
        }
        System.out.println();

        // go through the particle indexs and try to find their new assignments
        for (int i = 0; i < indexes.size(); i++) {
            int particle = indexes.get(i);
            int thrueTaskIndex = -1;
            ParticleValue pv = particleValueses[particle];
            for (int j = 0; j < assignments.length; j++) {
                if (assignments[j] == i) {
                    thrueTaskIndex = j % pv.getTotalTasks();
                    break;
                }
            }

            ParticleAssignment assignment = new ParticleAssignment(particle, i,
                    pv.getTaskId(), thrueTaskIndex);
            particleAssignments.addAssignment(assignment);
        }

        for (ParticleAssignment assignment : particleAssignments.getAssignments()) {
            System.out.format("pre task: %d prev i: %d new task: %d new i %d\n",
                    assignment.getPreviousTask(), assignment.getPreviousIndex(), assignment.getNewTask(), assignment.getNewIndex());;
        }

        return particleAssignments;
    }
}
