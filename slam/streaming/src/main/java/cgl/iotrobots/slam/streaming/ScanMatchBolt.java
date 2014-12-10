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
import cgl.iotrobots.slam.streaming.msgs.ParticleMaps;
import cgl.iotrobots.slam.streaming.msgs.ParticleValue;
import cgl.iotrobots.slam.streaming.rabbitmq.*;
import com.esotericsoftware.kryo.Kryo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This bolt is responsible for calculating for the assigned particles,
 * and then distribute the particles to re-sampler
 */
public class ScanMatchBolt extends BaseRichBolt {
    private Logger LOG = LoggerFactory.getLogger(ScanMatchBolt.class);

    private DistributedScanMatcher gfsp = null;

    private OutputCollector outputCollector;

    private TopologyContext topologyContext;

    /** Assignment are broad cast, so will create a broadcast receiver */
    private RabbitMQReceiver assignmentReceiver;

    /** Partciles are sent directly, so we will create a direct receiver */
    private RabbitMQReceiver particleReceiver;

    private RabbitMQSender sender;

    private String url = "";

    private Kryo kryo;

    private MatchState state = MatchState.WAITING_FOR_READING;

    private enum MatchState {
        WAITING_FOR_READING,
        COMPUTING_INIT_READINGS,
        WAITING_FOR_PARTICLE_ASSIGNMENTS,
        DISTRIBUTED_NEW_ASSIGNMENTS,
        WAITING_FOR_NEW_PARTICLES,
        COMPUTING_NEW_PARTICLES,
    }

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        gfsp = new DistributedScanMatcher();
        this.outputCollector = outputCollector;
        this.topologyContext = topologyContext;
        this.kryo = new Kryo();
        try {
            this.assignmentReceiver = new RabbitMQReceiver(url, Constants.Messages.BROADCAST_EXCHANGE, true);
            this.particleReceiver = new RabbitMQReceiver(url, Constants.Messages.DIRECT_EXCHANGE);
            this.sender = new RabbitMQSender(url, Constants.Messages.BROADCAST_EXCHANGE);
            this.sender.open();

            this.assignmentReceiver.listen(new ParticleAssignmentHandler());



        } catch (Exception e) {
            LOG.error("failed to create the message assignmentReceiver", e);
            throw new RuntimeException(e);
        }
    }

    @Override
    public void execute(Tuple tuple) {
        if (state != MatchState.WAITING_FOR_READING) {
            // we ack the tuple and discard it, because we cannot process the tuple at this moment
            outputCollector.ack(tuple);
            return;
        }

        Object val = tuple.getValueByField(Constants.Fields.LASER_SCAN_TUPLE);
        RangeReading reading;
        if (!(val instanceof RangeReading)) {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }
        int totalTasks = topologyContext.getComponentTasks(topologyContext.getThisComponentId()).size();
        reading = (RangeReading) val;

        // now we will start the computation
        state = MatchState.COMPUTING_INIT_READINGS;
        gfsp.processScan(reading, 0);

        // now distribute the particles to the bolts
        List<Integer> activeParticles = gfsp.getActiveParticles();
        List<Particle> particles = gfsp.getParticles();

        state = MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS;

        // after the computation we are going to create a new object without the map and nodes in particle and emit it
        // these will be used by the re sampler to re sample particles
        for (int index : activeParticles) {
            Particle particle = particles.get(index);

            int taskId = topologyContext.getThisTaskIndex();
            ParticleValue particleValue = new ParticleValue(taskId, index, totalTasks, particle.pose,
                    particle.previousPose, particle.weight,
                    particle.weightSum, particle.gweight, particle.previousIndex, particle.node);
            List<Object> emit = new ArrayList<Object>();
            emit.add(particleValue);
            outputCollector.emit(emit);
        }
    }

    /**
     * We will output the sensorId, time and particle value
     * @param outputFieldsDeclarer output
     */
    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new backtype.storm.tuple.Fields(Constants.Fields.SENSOR_ID_FIELD,
                Constants.Fields.TIME_FIELD,
                Constants.Fields.PARTICLE_VALUE));
    }

    private class ParticleAssignmentHandler implements MessageHandler {
        @Override
        public Map<String, String> getProperties() {
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, Constants.Messages.PARTICLE_ASSIGNMENT_ROUTING_KEY);

            return props;
        }

        @Override
        public void onMessage(Message message) {
            byte []body = message.getBody();
            if (state == MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS) {
                try {
                    ParticleAssignments assignments = (ParticleAssignments) Utils.deSerialize(kryo, body, ParticleAssignments.class);
                    // now go through the assignments and send them to the bolts directly
                    distributeAssignments(assignments);
                    state = MatchState.WAITING_FOR_NEW_PARTICLES;
                } catch (Exception e) {
                    LOG.error("Failed to deserialize assignment", e);
                }
            } else if (state == MatchState.WAITING_FOR_NEW_PARTICLES) {

            } else {
                LOG.error("Received message when we are in an unexpected state {}", state);
            }
        }
    }

    private void distributeAssignments(ParticleAssignments assignments) {
        List<ParticleAssignment> assignmentList = assignments.getAssignments();
        int taskId = topologyContext.getThisTaskIndex();
        for (int i = 0; i < assignmentList.size(); i++) {
            ParticleAssignment assignment = assignmentList.get(i);

            if (assignment.getPreviousTask() == taskId) {
                int previousIndex = assignment.getPreviousIndex();
                if (gfsp.getActiveParticles().contains(assignment.getPreviousIndex())) {
                    Particle p = gfsp.getParticles().get(previousIndex);
                    // create a new ParticleMaps
                    ParticleMaps particleMaps = new ParticleMaps(p.getMap(), p.getNode(), assignment.getNewIndex(), assignment.getNewTask());

                } else {
                    LOG.error("The particle {} is not in this bolt's active list, something is wrong", assignment.getPreviousIndex());
                }
            }
        }
    }
}
