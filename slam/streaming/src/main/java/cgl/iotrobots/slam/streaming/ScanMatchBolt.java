package cgl.iotrobots.slam.streaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.slam.core.GFSConfiguration;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.app.Position;
import cgl.iotrobots.slam.core.grid.Array2D;
import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.grid.HierarchicalArray2D;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.gridfastsalm.TNode;
import cgl.iotrobots.slam.core.scanmatcher.PointAccumulator;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.sensor.RangeSensor;
import cgl.iotrobots.slam.core.sensor.Sensor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.core.utils.DoublePoint;
import cgl.iotrobots.slam.core.utils.IntPoint;
import cgl.iotrobots.slam.streaming.msgs.*;
import cgl.iotrobots.utils.rabbitmq.*;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import com.esotericsoftware.kryo.Kryo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

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

    /** Particles are sent directly, so we will create a direct receiver */
    private RabbitMQReceiver particleReceiver;

    private RabbitMQReceiver particleValueReceiver;

    private RabbitMQSender particleSender;

    private String url = "amqp://localhost:5672";

    private Kryo kryo;

    private MatchState state = MatchState.WAITING_FOR_READING;

    private int expectingParticleMaps = 0;
    private int expectingParticleValues = 0;

    private ParticleAssignments assignments = null;

    private enum MatchState {
        WAITING_FOR_READING,
        COMPUTING_INIT_READINGS,
        WAITING_FOR_PARTICLE_ASSIGNMENTS_AND_NEW_PARTICLES,
        COMPUTING_NEW_PARTICLES,
    }

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        gfsp = new DistributedScanMatcher();
        this.outputCollector = outputCollector;
        this.topologyContext = topologyContext;
        this.kryo = new Kryo();
        this.kryo.register(Object[][].class);
        registerClasses(kryo);
        // read the configuration of the scanmatcher from topology.xml
        StreamTopologyBuilder streamTopologyBuilder = new StreamTopologyBuilder();
        StreamComponents components = streamTopologyBuilder.buildComponents();
        // use the configuration to create the scanmatcher
        GFSConfiguration cfg = ConfigurationBuilder.getConfiguration(components.getConf());
        gfsp = ProcessorFactory.createMatcher(cfg);

        this.url = (String) components.getConf().get(Constants.RABBITMQ_URL);
        try {
            // we use broadcast to receive assignments
            this.assignmentReceiver = new RabbitMQReceiver(url, Constants.Messages.BROADCAST_EXCHANGE, true);
            // we use direct exchange to receive particle maps
            this.particleReceiver = new RabbitMQReceiver(url, Constants.Messages.DIRECT_EXCHANGE);
            // we use direct to receive particle values
            this.particleValueReceiver = new RabbitMQReceiver(url, Constants.Messages.DIRECT_EXCHANGE);
            // we use direct to send the new particle maps
            this.particleSender = new RabbitMQSender(url, Constants.Messages.DIRECT_EXCHANGE);
            this.particleSender.open();

            this.assignmentReceiver.listen(new ParticleAssignmentHandler());
            this.particleReceiver.listen(new MapHandler());
            this.particleValueReceiver.listen(new ParticleValueHandler());
        } catch (Exception e) {
            LOG.error("failed to create the message assignmentReceiver", e);
            throw new RuntimeException(e);
        }

        // set the initial particles
        int totalTasks = topologyContext.getComponentTasks(topologyContext.getThisComponentId()).size();
        int taskId = topologyContext.getThisTaskIndex();
        int noOfParticles = cfg.getNoOfParticles() / totalTasks;
        int remainder = cfg.getNoOfParticles() % totalTasks;
        if (remainder > 0 && remainder <= taskId) {
            noOfParticles += 1;
        }
        List<Integer> activeParticles = gfsp.getActiveParticles();
        for (int i = 0; i < noOfParticles; i++) {
            activeParticles.add(i + taskId * noOfParticles);
        }
    }

    double[] plainReading;
    RangeReading rangeReading;
    LaserScan scan;
    Object time;
    Object sensorId;

    @Override
    public void execute(Tuple tuple) {
        if (state != MatchState.WAITING_FOR_READING) {
            // we ack the tuple and discard it, because we cannot process the tuple at this moment
            outputCollector.ack(tuple);
            return;
        }
        int taskId = topologyContext.getThisTaskIndex();
        outputCollector.ack(tuple);
        time = tuple.getValueByField(Constants.Fields.TIME_FIELD);
        sensorId = tuple.getValueByField(TransportConstants.SENSOR_ID);

        Object val = tuple.getValueByField(Constants.Fields.LASER_SCAN_FIELD);
        if (!(val instanceof byte [])) {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }
        scan = (LaserScan) Utils.deSerialize(kryo, (byte [])val, LaserScan.class);
        RangeReading reading;


        int totalTasks = topologyContext.getComponentTasks(topologyContext.getThisComponentId()).size();
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
        gfsp.setSensorMap(smap);

        rangeReading = reading;
        plainReading = new double[gfsp.beams];
        for (int i = 0; i < gfsp.beams; i++) {
            plainReading[i] = reading.get(i);
        }

        // now we will start the computation
        LOG.info("taskId {}: Changing state to COMPUTING_INIT_READINGS", taskId);
        state = MatchState.COMPUTING_INIT_READINGS;
        if (!gfsp.processScan(reading, 0)) {
            return;
        }

        // now distribute the particles to the bolts
        List<Integer> activeParticles = gfsp.getActiveParticles();
        List<Particle> particles = gfsp.getParticles();

        LOG.info("taskId {}: execute: changing state to WAITING_FOR_PARTICLE_ASSIGNMENTS_AND_NEW_PARTICLES", taskId);
        state = MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS_AND_NEW_PARTICLES;

        // after the computation we are going to create a new object without the map and nodes in particle and emit it
        // these will be used by the re sampler to re sample particles
        for (int index : activeParticles) {
            Particle particle = particles.get(index);


            ParticleValue particleValue = new ParticleValue(taskId, index, totalTasks, particle.pose,
                    particle.previousPose, particle.weight,
                    particle.weightSum, particle.gweight, particle.previousIndex, particle.node);
            List<Object> emit = new ArrayList<Object>();
            emit.add(particleValue);
            emit.add(scan);
            emit.add(sensorId);
            emit.add(time);
            outputCollector.emit(Constants.Fields.PARTICLE_STREAM, emit);
        }
    }

    private void registerClasses(Kryo kryo) {
        kryo.register(DoublePoint.class);
        kryo.register(IntPoint.class);
        kryo.register(Particle.class);
        kryo.register(GMap.class);
        kryo.register(Array2D.class);
        kryo.register(HierarchicalArray2D.class);
        kryo.register(TNode.class);
        kryo.register(DoubleOrientedPoint.class);
        kryo.register(Particle.class);
        kryo.register(PointAccumulator.class);
        kryo.register(HierarchicalArray2D.class);
        kryo.register(Array2D.class);
        kryo.register(Position.class);
        kryo.register(Object[][].class);
        kryo.register(TransferMap.class);
        kryo.register(ParticleMaps.class);
        kryo.register(MapCell.class);
    }

    /**
     * We will output the sensorId, time and particle value
     * @param outputFieldsDeclarer output
     */
    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constants.Fields.PARTICLE_STREAM, new Fields(Constants.Fields.PARTICLE_VALUE_FIELD,
                Constants.Fields.LASER_SCAN_FIELD,
                Constants.Fields.SENSOR_ID_FIELD,
                Constants.Fields.TIME_FIELD));

        outputFieldsDeclarer.declareStream(Constants.Fields.MAP_STREAM, new Fields(
                Constants.Fields.PARTICLE_VALUE_FIELD,
                Constants.Fields.PARTICLE_MAP_FIELD,
                Constants.Fields.LASER_SCAN_FIELD,
                Constants.Fields.SENSOR_ID_FIELD,
                Constants.Fields.TIME_FIELD));
    }

    /** We are going to keep the particles maps until we get an assignment */
    private List<ParticleMaps> particleMapses = new ArrayList<ParticleMaps>();

    private class MapHandler implements MessageHandler {
        @Override
        public Map<String, String> getProperties() {
            int taskId = topologyContext.getThisTaskIndex();
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, Constants.Messages.PARTICLE_MAP_ROUTING_KEY + "_" + taskId);

            return props;
        }

        @Override
        public void onMessage(Message message) {
            int taskId = topologyContext.getThisTaskIndex();
            byte []body = message.getBody();
            if (state == MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS_AND_NEW_PARTICLES) {
                try {
                    LOG.info("taskId {}: Received particle map", taskId);
                    ParticleMaps pm = (ParticleMaps) Utils.deSerialize(kryo, body, ParticleMaps.class);
                    // first we need to determine the expected new maps for this particle
                    if (assignments != null) {
                        // first check weather particleMapses not empty. If not empty handle those first
                        if (!particleMapses.isEmpty()) {
                            for (ParticleMaps existingPm : particleMapses) {
                                addMaps(existingPm);
                            }
                            // after handling the temp values, we'll clear the buffer
                            particleMapses.clear();
                        }
                        // now go through the assignments and send them to the bolts directly
                        addMaps(pm);

                        // we have received all the particles we need to do the processing after resampling
                        if (expectingParticleMaps == 0 && expectingParticleValues == 0) {
                            state = MatchState.COMPUTING_NEW_PARTICLES;
                            LOG.info("taskId {}: Changing state to COMPUTING_NEW_PARTICLES", taskId);
                            gfsp.processAfterReSampling(plainReading);

                            emitParticleForMap();

                            state = MatchState.WAITING_FOR_READING;
                            LOG.info("taskId {}: Changing state to WAITING_FOR_READING", taskId);
                        }
                    } else {
                        // because we haven't received the assignments yet, we will keep the values temporaly in this list
                        particleMapses.add(pm);
                    }
                } catch (Exception e) {
                    LOG.error("taskId {}: Failed to deserialize map", taskId, e);
                }
            } else {
                LOG.error("taskId {}: Received message when we are in an unexpected state {}", taskId, state);
            }
        }
    }

    private void emitParticleForMap() {
        int particle = gfsp.getBestParticleIndex();
        Particle best = gfsp.getParticles().get(particle);
        List<Object> emit = new ArrayList<Object>();

        ParticleValue particleValue = new ParticleValue(-1, -1, -1, best.pose,
                best.previousPose, best.weight,
                best.weightSum, best.gweight, best.previousIndex, best.node);
        TransferMap map = Utils.createTransferMap(best.getMap());

        emit.add(particleValue);
        emit.add(map);
        emit.add(scan);
        emit.add(sensorId);
        emit.add(time);

        outputCollector.emit(Constants.Fields.MAP_STREAM, emit);
    }

    private boolean assignmentExists(int task, int index, List<ParticleAssignment> assignmentList) {
        for (ParticleAssignment assignment : assignmentList) {
            if (assignment.getNewTask() == task && assignment.getNewIndex() == index) {
                return true;
            }
        }
        return true;
    }

    private class ParticleAssignmentHandler implements MessageHandler {
        @Override
        public Map<String, String> getProperties() {
            int taskId = topologyContext.getThisTaskIndex();
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, Constants.Messages.PARTICLE_ASSIGNMENT_ROUTING_KEY);
            return props;
        }

        @Override
        public void onMessage(Message message) {
            int taskId = topologyContext.getThisTaskIndex();
            byte []body = message.getBody();
            if (state == MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS_AND_NEW_PARTICLES) {
                try {
                    LOG.info("taskId {}: Received particle assignment", taskId);
                    ParticleAssignments assignments = (ParticleAssignments) Utils.deSerialize(kryo, body, ParticleAssignments.class);


                    // if we have resampled ditributed the assignments
                    if (assignments.isReSampled()) {
                        // now go through the assignments and send them to the bolts directly
                        computeExpectedParticles(assignments);
                        distributeAssignments(assignments);
                        gfsp.getActiveParticles().clear();
                        // we are going to keep the assignemtns so that we can check the receiving particles
                        ScanMatchBolt.this.assignments = assignments;
                    } else {
                        // we are going to keep the assignemtns so that we can check the receiving particles
                        ScanMatchBolt.this.assignments = assignments;
                        expectingParticleMaps = 0;
                        expectingParticleValues = 0;
                        LOG.info("taskId {}: Changing state to COMPUTING_NEW_PARTICLES", taskId);
                        state = MatchState.COMPUTING_NEW_PARTICLES;

                        // we need to do the post processing we need for the particles
                        gfsp.postProcessingWithoutReSampling(plainReading, rangeReading);
                        emitParticleForMap();
                        LOG.info("taskId {}: Changing state to WAITING_FOR_READING", taskId);
                        state = MatchState.WAITING_FOR_READING;
                    }
                } catch (Exception e) {
                    LOG.error("taskId {}: Failed to deserialize assignment", taskId, e);
                }
            } else {
                LOG.error("Received message when we are in an unexpected state {}", state);
            }
        }
    }

    private void computeExpectedParticles(ParticleAssignments assignments) {
        List<ParticleAssignment> assignmentList = assignments.getAssignments();
        int taskId = topologyContext.getThisTaskIndex();
        for (int i = 0; i < assignmentList.size(); i++) {
            ParticleAssignment assignment = assignmentList.get(i);
            if (assignment.getNewTask() == taskId) {
                expectingParticleMaps++;
                expectingParticleValues++;
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
                    ParticleMaps particleMaps = new ParticleMaps(Utils.createTransferMap(p.getMap()),
                            assignment.getNewIndex(), assignment.getNewTask());

                    byte []b = Utils.serialize(kryo, particleMaps);
                    Message message = new Message(b, new HashMap<String, Object>());
                    LOG.info("Sending particle map to {}", assignment.getNewTask());
                    try {
                        particleSender.send(message, Constants.Messages.PARTICLE_MAP_ROUTING_KEY + "_" + assignment.getNewTask());
                    } catch (Exception e) {
                        LOG.error("taskId {}: Failed to send the new particle map", taskId, e);
                    }
                } else {
                    LOG.error("taskId {}: The particle {} is not in this bolt's active list, something is wrong", taskId,
                            assignment.getPreviousIndex());
                }
            }
        }
    }

    private List<ParticleValue> particleValues = new ArrayList<ParticleValue>();

    private class ParticleValueHandler implements MessageHandler {
        @Override
        public Map<String, String> getProperties() {
            int taskId = topologyContext.getThisTaskIndex();
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, Constants.Messages.PARTICLE_VALUE_ROUTING_KEY + "_" + taskId);
            return props;
        }

        @Override
        public void onMessage(Message message) {
            int taskId = topologyContext.getThisTaskIndex();
            byte []body = message.getBody();
            if (state == MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS_AND_NEW_PARTICLES) {
                try {
                    LOG.info("taskId {}: Received particle value", taskId);
                    ParticleValue pm = (ParticleValue) Utils.deSerialize(kryo, body, ParticleValue.class);
                    // first we need to determine the expected new maps for this particle
                    if (assignments != null) {
                        // first check weather particleMapses not empty. If not empty handle those first
                        if (!particleMapses.isEmpty()) {
                            for (ParticleValue existingPm : particleValues) {
                                addParticle(existingPm);
                            }

                            particleMapses.clear();
                        }
                        // now go through the assignments and send them to the bolts directly
                        addParticle(pm);

                        // we have received all the particles we need to do the processing after resampling
                        if (expectingParticleValues == 0 && expectingParticleMaps == 0) {
                            state = MatchState.COMPUTING_NEW_PARTICLES;
                            LOG.info("taskId {}: Changing state to COMPUTING_NEW_PARTICLES", taskId);
                            gfsp.processAfterReSampling(plainReading);
                            emitParticleForMap();
                            state = MatchState.WAITING_FOR_READING;
                            LOG.info("taskId {}: Changing state to WAITING_FOR_READING", taskId);
                        }
                    } else {
                        // because we haven't received the assignments yet, we will keep the values temporaly in this list
                        particleValues.add(pm);
                    }
                } catch (Exception e) {
                    LOG.error("taskId {}: Failed to deserialize assignment", taskId, e);
                }
            } else {
                LOG.error("taskId {}: Received message when we are in an unexpected state {}", taskId, state);
            }
        }
    }

    /**
     * The particle values calculated after the resampling
     * @param value values
     */
    private void addParticle(ParticleValue value) {
        List<ParticleAssignment> assignmentList = assignments.getAssignments();
        int taskId = topologyContext.getThisTaskIndex();
        boolean found = assignmentExists(value.getTaskId(), value.getIndex(), assignmentList);

        if (!found) {
            String msg = "taskId " + taskId + ": We got a particle that doesn't belong here";
            LOG.error(msg);
            throw new RuntimeException(msg);
        }

        int newIndex = value.getIndex();
        Particle p = gfsp.getParticles().get(newIndex);

        // populate particle using particle values
        Utils.createParticle(value, p);

        gfsp.getActiveParticles().add(newIndex);
        // add the new particle index
        if (!gfsp.getActiveParticles().contains(newIndex)) {
            gfsp.getActiveParticles().add(newIndex);
        }

        // we have received one particle
        expectingParticleValues--;
        LOG.info("Expecting particle values {}", expectingParticleMaps);
    }



    /**
     * Add a new partcle maps and node tree to the particle
     * @param particleMaps the map and node tree
     */
    private void addMaps(ParticleMaps particleMaps) {
        List<ParticleAssignment> assignmentList = assignments.getAssignments();
        int taskId = topologyContext.getThisTaskIndex();
        boolean found = assignmentExists(particleMaps.getTask(), particleMaps.getIndex(), assignmentList);

        if (!found) {
            String msg = "taskId " + taskId + ": We got a particle that doesn't belong here";
            LOG.error(msg);
            throw new RuntimeException(msg);
        }

        int newIndex = particleMaps.getIndex();
        Particle p = gfsp.getParticles().get(newIndex);

        p.setMap(Utils.createGMap(particleMaps.getMap()));

        // add the new particle index
        if (!gfsp.getActiveParticles().contains(newIndex)) {
            gfsp.getActiveParticles().add(newIndex);
        }

        // we have received one particle
        expectingParticleMaps--;
        LOG.info("Expecting particle maps {}", expectingParticleMaps);
    }

    @Override
    public void cleanup() {
        super.cleanup();

        LOG.info("Shutting down the bolt");
    }


}
