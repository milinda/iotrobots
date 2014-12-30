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
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.sensor.RangeSensor;
import cgl.iotrobots.slam.core.sensor.Sensor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.streaming.msgs.*;
import cgl.iotrobots.utils.rabbitmq.*;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import com.esotericsoftware.kryo.Kryo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

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
    private RabbitMQReceiver particleMapReceiver;

    private RabbitMQReceiver particleValueReceiver;

    private RabbitMQSender particleSender;

    private String url = "amqp://localhost:5672";

    private Kryo kryoPVReading;

    private Kryo kryoMapReading;

    private Kryo kryoMapWriting;

    private Kryo kryoAssignReading;

    private Kryo kryoLaserReading;

    private Kryo kryoBestParticle;

    private volatile MatchState state = MatchState.WAITING_FOR_READING;

    private int expectingParticleMaps = 0;
    private int expectingParticleValues = 0;

    private ParticleAssignments assignments = null;

    private Lock lock = new ReentrantLock();

    private enum MatchState {
        WAITING_FOR_READING,
        COMPUTING_INIT_READINGS,
        WAITING_FOR_PARTICLE_ASSIGNMENTS,
        WAITING_FOR_NEW_PARTICLES,
        COMPUTING_NEW_PARTICLES,
    }

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        gfsp = new DistributedScanMatcher();
        this.outputCollector = outputCollector;
        this.topologyContext = topologyContext;
        this.kryoAssignReading = new Kryo();
        this.kryoPVReading = new Kryo();
        this.kryoMapWriting = new Kryo();
        this.kryoMapReading = new Kryo();
        this.kryoLaserReading = new Kryo();
        this.kryoBestParticle = new Kryo();

        Utils.registerClasses(kryoAssignReading);
        Utils.registerClasses(kryoPVReading);
        Utils.registerClasses(kryoMapWriting);
        Utils.registerClasses(kryoMapReading);
        Utils.registerClasses(kryoLaserReading);
        Utils.registerClasses(kryoBestParticle);

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
            this.particleMapReceiver = new RabbitMQReceiver(url, Constants.Messages.DIRECT_EXCHANGE);
            // we use direct to receive particle values
            this.particleValueReceiver = new RabbitMQReceiver(url, Constants.Messages.DIRECT_EXCHANGE);
            // we use direct to send the new particle maps
            this.particleSender = new RabbitMQSender(url, Constants.Messages.DIRECT_EXCHANGE);
            this.particleSender.open();

            this.assignmentReceiver.listen(new ParticleAssignmentHandler());
            this.particleMapReceiver.listen(new MapHandler());
            this.particleValueReceiver.listen(new ParticleValueHandler());
        } catch (Exception e) {
            LOG.error("failed to create the message assignmentReceiver", e);
            throw new RuntimeException(e);
        }

        // set the initial particles
        int totalTasks = topologyContext.getComponentTasks(topologyContext.getThisComponentId()).size();
        int taskId = topologyContext.getThisTaskIndex();
        int noOfParticles = computeParticlesForTask(cfg, totalTasks, taskId);

        int previousTotal = 0;
        for (int i = 0; i < taskId; i++) {
            previousTotal += computeParticlesForTask(cfg, totalTasks, i);
        }

        List<Integer> activeParticles = gfsp.getActiveParticles();
        for (int i = 0; i < noOfParticles; i++) {
            activeParticles.add(i + previousTotal);
        }

        LOG.info("taskId {}: no of active particles {}", taskId, activeParticles.size());
    }

    /**
     * Given a task id compute the no of particles
     * @param cfg configuration
     * @param totalTasks total tasks
     * @param taskId task id
     * @return no of particles for this task
     */
    private int computeParticlesForTask(GFSConfiguration cfg, int totalTasks, int taskId) {
        int noOfParticles = cfg.getNoOfParticles() / totalTasks;
        int remainder = cfg.getNoOfParticles() % totalTasks;
        if (remainder > 0 &&  remainder < (totalTasks - taskId)) {
            noOfParticles += 1;
        }
        return noOfParticles;
    }

    double[] plainReading;
    RangeReading rangeReading;
    LaserScan scan;
    Object time;
    Object sensorId;
    long lastMessageTime;
    long lastComputationTime;
    // flag to be set when this bolt is has the best particle

    @Override
    public void execute(Tuple tuple) {
        if (state != MatchState.WAITING_FOR_READING) {
            // we ack the tuple and discard it, because we cannot process the tuple at this moment
            outputCollector.ack(tuple);
            return;
        }

        // check weather this tuple came between the last computation time. if so discard it

        long t0 = System.currentTimeMillis();
        int taskId = topologyContext.getThisTaskIndex();

        time = tuple.getValueByField(Constants.Fields.TIME_FIELD);
        sensorId = tuple.getValueByField(TransportConstants.SENSOR_ID);

        long currentMessageTime = Long.parseLong(time.toString());
        // if this message came within that window, discard it
        // this will allow us to keep track of the current interval
        if (currentMessageTime < lastComputationTime * 2 + lastMessageTime) {
            outputCollector.ack(tuple);
            return;
        }

        Object val = tuple.getValueByField(Constants.Fields.LASER_SCAN_FIELD);
        if (!(val instanceof byte [])) {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }
        lock.lock();
        try {
            scan = (LaserScan) Utils.deSerialize(kryoLaserReading, (byte[]) val, LaserScan.class);
        } finally {
            lock.unlock();
        }
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
        state = MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS;

        // after the computation we are going to create a new object without the map and nodes in particle and emit it
        // these will be used by the re sampler to re sample particles
        LOG.info("taskId {}: no of active particles {}", taskId, activeParticles.size());
        List<ParticleValue> pvs = new ArrayList<ParticleValue>();
        for (int i = 0; i < activeParticles.size(); i++) {
            int index = activeParticles.get(i);
            Particle particle = particles.get(index);
//            ParticleValue particleValue = new ParticleValue(taskId, index, totalTasks, particle.pose,
//                    particle.previousPose, particle.weight,
//                    particle.weightSum, particle.gweight, particle.previousIndex, particle.node);
            ParticleValue particleValue = Utils.createParticleValue(particle, taskId, index, totalTasks);
            pvs.add(particleValue);

        }
        List<Object> emit = new ArrayList<Object>();
        emit.add(pvs);
        emit.add(scan);
        emit.add(sensorId);
        emit.add(time);
        outputCollector.emit(Constants.Fields.PARTICLE_STREAM, emit);

        outputCollector.ack(tuple);
        // we compute the last computation time
        lastComputationTime = System.currentTimeMillis() - t0;
        lastMessageTime = Long.parseLong(time.toString());
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
//                Constants.Fields.PARTICLE_MAP_FIELD,
                Constants.Fields.LASER_SCAN_FIELD,
                Constants.Fields.SENSOR_ID_FIELD,
                Constants.Fields.TIME_FIELD));

        outputFieldsDeclarer.declareStream(Constants.Fields.BEST_PARTICLE_STREAM,
                new Fields("body", Constants.Fields.SENSOR_ID_FIELD, Constants.Fields.TIME_FIELD));
    }

    /** We are going to keep the particles maps until we get an assignment */
    private List<ParticleMaps> particleMapses = new ArrayList<ParticleMaps>();

    private class MapHandler implements MessageHandler {
        @Override
        public Map<String, String> getProperties() {
            int taskId = topologyContext.getThisTaskIndex();
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_QUEUE, Constants.Messages.PARTICLE_MAP_ROUTING_KEY + "_" + taskId);
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, Constants.Messages.PARTICLE_MAP_ROUTING_KEY + "_" + taskId);
            return props;
        }

        @Override
        public void onMessage(Message message) {
            int taskId = topologyContext.getThisTaskIndex();
            byte []body = message.getBody();
            LOG.info("taskId {}: Received particle map", taskId);
            ParticleMaps pm = (ParticleMaps) Utils.deSerialize(kryoMapReading, body, ParticleMaps.class);
            if (state == MatchState.WAITING_FOR_NEW_PARTICLES) {
                try {
                    // first we need to determine the expected new maps for this particle
                    // first check weather particleMapses not empty. If not empty handle those first
                    processReceivedMaps("map");
                    processReceivedValues("map");
                    // now go through the assignments and send them to the bolts directly
                    addMaps(pm, "map");

                    // we have received all the particles we need to do the processing after resampling
                    postProcessingAfterReceiveAll(taskId, "map", assignments.getBestParticle());
                } catch (Exception e) {
                    LOG.error("taskId {}: Failed to deserialize map", taskId, e);
                }
            } else if (state == MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS) {
                // because we haven't received the assignments yet, we will keep the values temporaly in this list
                LOG.info("taskId {}: Adding map state {}", taskId, state);
                lock.lock();
                try {
                    particleMapses.add(pm);
                    if (state != MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS) {
                        postProcessingAfterReceiveAll(taskId, "adding map", assignments.getBestParticle());
                    }
                } finally {
                    lock.unlock();
                }
            } else {
                LOG.error("taskId {}: Received message when we are in an unexpected state {}", taskId, state);
            }
        }
    }

    private void postProcessingAfterReceiveAll(int taskId, String origin, boolean resampled, int best) {
        if (state == MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS) {
            lock.lock();
            try {
                LOG.info("taskId {}: {} Changing state to WAITING_FOR_NEW_PARTICLES", origin, taskId);
                state = MatchState.WAITING_FOR_NEW_PARTICLES;
            } finally {
                lock.unlock();
            }
        }

        processReceivedMaps(origin);
        processReceivedValues(origin);

        if (expectingParticleMaps == 0 && expectingParticleValues == 0) {
            state = MatchState.COMPUTING_NEW_PARTICLES;
            LOG.info("taskId {}: Map Handler Changing state to COMPUTING_NEW_PARTICLES", taskId);
            if (resampled) {
                gfsp.processAfterReSampling(plainReading);
            } else {
                gfsp.postProcessingWithoutReSampling(plainReading, rangeReading);

            }

            // find the particle with the best index
            // find the particle with the best index
            if (gfsp.getActiveParticles().contains(best)) {
                emitParticleForMap(best);
            }

            this.assignments = null;
            state = MatchState.WAITING_FOR_READING;
            LOG.info("taskId {}: Changing state to WAITING_FOR_READING", taskId);
        }
    }

    private void postProcessingAfterReceiveAll(int taskId, String origin, int best) {
        if (state == MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS) {
            lock.lock();
            try {
                LOG.info("taskId {}: {} Changing state to WAITING_FOR_NEW_PARTICLES", origin, taskId);
                state = MatchState.WAITING_FOR_NEW_PARTICLES;
            } finally {
                lock.unlock();
            }
        }

        processReceivedMaps(origin);
        processReceivedValues(origin);

        if (expectingParticleMaps == 0 && expectingParticleValues == 0) {
            state = MatchState.COMPUTING_NEW_PARTICLES;
            LOG.info("taskId {}: Map Handler Changing state to COMPUTING_NEW_PARTICLES", taskId);
            gfsp.processAfterReSampling(plainReading);

            // find the particle with the best index
            if (gfsp.getActiveParticles().contains(best)) {
                emitParticleForMap(best);
            }

            this.assignments = null;
            state = MatchState.WAITING_FOR_READING;
            LOG.info("taskId {}: Changing state to WAITING_FOR_READING", taskId);
        }
    }

    private void processReceivedMaps(String origin) {
        lock.lock();
        try {
            if (!particleMapses.isEmpty()) {
                for (ParticleMaps existingPm : particleMapses) {
                    addMaps(existingPm, origin);
                }
                // after handling the temp values, we'll clear the buffer
                particleMapses.clear();
            }
        } finally {
            lock.unlock();
        }
    }

    private void emitParticleForMap(int index) {
        LOG.info("Emit for map");
        // tru tp see weather this bolt has the best particle
//        ParticleValue pv = particleValues.get(index);
//        LOG.error("Emit for map, best particle");
        Particle best = gfsp.getParticles().get(index);
        List<Object> emit = new ArrayList<Object>();

        ParticleValue particleValue = Utils.createParticleValue(best, -1, -1, -1);
        LOG.error("Emit for map, transfermap");
//        TransferMap map = Utils.createTransferMap(best.getMap());

        emit.add(particleValue);
//        emit.add(map);
        emit.add(scan);
        emit.add(sensorId);
        emit.add(time);
        LOG.error("Emit for map, collector");
        outputCollector.emit(Constants.Fields.MAP_STREAM, emit);

        List<Object> emitValue = new ArrayList<Object>();
        emitValue.add(Utils.serialize(kryoBestParticle, ParticleValue.class));
        emitValue.add(sensorId);
        emitValue.add(time);
        outputCollector.emit(Constants.Fields.BEST_PARTICLE_STREAM, emitValue);
//        ParticleValue particleValue = new ParticleValue(-1, -1, -1, best.pose,
//                best.previousPose, best.weight,
//                best.weightSum, best.gweight, best.previousIndex, best.node);

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
            props.put(MessagingConstants.RABBIT_QUEUE, Constants.Messages.PARTICLE_ASSIGNMENT_ROUTING_KEY + "_" + taskId);
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, Constants.Messages.PARTICLE_ASSIGNMENT_ROUTING_KEY);
            return props;
        }

        @Override
        public void onMessage(Message message) {
            int taskId = topologyContext.getThisTaskIndex();
            byte []body = message.getBody();
            if (state == MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS) {
                try {
                    LOG.info("taskId {}: Received particle assignment", taskId);
                    ParticleAssignments assignments = (ParticleAssignments) Utils.deSerialize(kryoAssignReading, body, ParticleAssignments.class);
                    LOG.info("taskId {}: Best particle index {}", taskId, assignments.getBestParticle());
                    // if we have resampled ditributed the assignments
                    if (assignments.isReSampled()) {
                        // now go through the assignments and send them to the bolts directly
                        computeExpectedParticles(assignments);
                        distributeAssignments(assignments);
                        LOG.info("taskId {}: Clearing active particles", taskId);
                        gfsp.clearActiveParticles();

                        // we are going to keep the assignemtns so that we can check the receiving particles
                        ScanMatchBolt.this.assignments = assignments;
                        processReceivedValues("assign");
                        processReceivedMaps("assign-maps");
                        postProcessingAfterReceiveAll(taskId, "assign-all", assignments.getBestParticle());
                    } else {
                        // we are going to keep the assignemtns so that we can check the receiving particles
                        ScanMatchBolt.this.assignments = assignments;
                        expectingParticleValues = 0;
                        expectingParticleMaps = 0;
                        postProcessingAfterReceiveAll(taskId, "assign", false, assignments.getBestParticle());
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
        // we set them to 0 as a fallback method, these should be set to 0 automatically
        expectingParticleValues = 0;
        expectingParticleMaps = 0;
        for (int i = 0; i < assignmentList.size(); i++) {
            ParticleAssignment assignment = assignmentList.get(i);
            if (assignment.getNewTask() == taskId) {
                expectingParticleValues++;
                // we only expects maps from other bolt tasks
                if (assignment.getPreviousTask() != taskId) {
                    expectingParticleMaps++;
                }
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
                    // send the particle over rabbitmq if this is a different task
                    if (assignment.getNewTask() != taskId) {
                        Particle p = gfsp.getParticles().get(previousIndex);
                        // create a new ParticleMaps
                        ParticleMaps particleMaps = new ParticleMaps(Utils.createTransferMap(p.getMap()),
                                assignment.getNewIndex(), assignment.getNewTask());

                        byte[] b = Utils.serialize(kryoMapWriting, particleMaps);
                        Message message = new Message(b, new HashMap<String, Object>());
                        LOG.info("Sending particle map to {}", assignment.getNewTask());
                        try {
                            particleSender.send(message, Constants.Messages.PARTICLE_MAP_ROUTING_KEY + "_" + assignment.getNewTask());
                        } catch (Exception e) {
                            LOG.error("taskId {}: Failed to send the new particle map", taskId, e);
                        }
                    } else {
                        // add the previous particles map to the new particles map
                        int newIndex = assignment.getNewIndex();
                        int prevIndex = assignment.getPreviousIndex();
                        Particle p = gfsp.getParticles().get(newIndex);
                        Particle pOld = gfsp.getParticles().get(prevIndex);

                        p.setMap(pOld.getMap());
                        // gfsp.getActiveParticles().add(newIndex);
                        // add the new particle index
                        gfsp.addActiveParticle(newIndex);
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
            props.put(MessagingConstants.RABBIT_QUEUE, Constants.Messages.PARTICLE_VALUE_ROUTING_KEY + "_" + taskId);
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, Constants.Messages.PARTICLE_VALUE_ROUTING_KEY + "_" + taskId);
            return props;
        }

        @Override
        public void onMessage(Message message) {
            int taskId = topologyContext.getThisTaskIndex();
            byte []body = message.getBody();
            ParticleValue pm = (ParticleValue) Utils.deSerialize(kryoPVReading, body, ParticleValue.class);
            LOG.info("taskId {}: Received particle value", taskId);
            if (state == MatchState.WAITING_FOR_NEW_PARTICLES) {
                try {
                    // first we need to determine the expected new maps for this particle
                    // first check weather particleMapses not empty. If not empty handle those first
                    processReceivedValues("value");
                    processReceivedMaps("value");
                    // now go through the assignments and send them to the bolts directly
                    addParticle(pm, "value");

                    // we have received all the particles we need to do the processing after resampling
                    postProcessingAfterReceiveAll(taskId, "assign", assignments.getBestParticle());
                } catch (Exception e) {
                    LOG.error("taskId {}: Failed to deserialize assignment", taskId, e);
                }
            } else if (state == MatchState.WAITING_FOR_PARTICLE_ASSIGNMENTS) {
                // first we need to determine the expected new maps for this particle
                // because we haven't received the assignments yet, we will keep the values temporaly in this list
                lock.lock();
                try {
                    particleValues.add(pm);
                } finally {
                    lock.unlock();
                }

            } else {
                LOG.error("taskId {}: Received message when we are in an unexpected state {}", taskId, state);
            }
        }
    }

    private void processReceivedValues(String origin) {
        lock.lock();
        try {
            if (!particleValues.isEmpty()) {
                for (ParticleValue existingPm : particleValues) {
                    addParticle(existingPm, origin);
                }
                particleValues.clear();
            }
        } finally {
            lock.unlock();
        }
    }

    /**
     * The particle values calculated after the resampling
     * @param value values
     */
    private void addParticle(ParticleValue value, String origin) {
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

        // gfsp.getActiveParticles().add(newIndex);
        // add the new particle index
        gfsp.addActiveParticle(newIndex);

        // we have received one particle
        expectingParticleValues--;
        LOG.info("taskId {}: Expecting particle values {} origin {}", taskId, expectingParticleValues, origin);
    }



    /**
     * Add a new partcle maps and node tree to the particle
     * @param particleMaps the map and node tree
     */
    private void addMaps(ParticleMaps particleMaps, String origin) {
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
        gfsp.addActiveParticle(newIndex);

        // we have received one particle
        expectingParticleMaps--;
        LOG.info("taskId {}: Expecting particle maps {} origin {}", taskId, expectingParticleMaps, origin);
    }

    @Override
    public void cleanup() {
        super.cleanup();

        LOG.info("Shutting down the bolt");
    }


}
