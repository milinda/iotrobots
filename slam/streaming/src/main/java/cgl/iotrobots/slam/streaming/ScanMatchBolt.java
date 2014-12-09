package cgl.iotrobots.slam.streaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.streaming.msgs.ParticleValue;
import cgl.iotrobots.slam.streaming.rabbitmq.RabbitMQReceiver;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
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

    private RabbitMQReceiver receiver;

    private String url = "";

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        gfsp = new DistributedScanMatcher();
        this.outputCollector = outputCollector;
        this.topologyContext = topologyContext;
        try {
            this.receiver = new RabbitMQReceiver(url, Constants.Messages.SLAM_EXCHANGE);
        } catch (Exception e) {
            LOG.error("failed to create the message receiver", e);
            throw new RuntimeException(e);
        }
    }

    @Override
    public void execute(Tuple tuple) {
        Object val = tuple.getValueByField(Constants.Fields.LASER_SCAN_TUPLE);
        RangeReading reading;
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
            ParticleValue particleValue = new ParticleValue(taskId, index, totalTasks, particle.pose,
                    particle.previousPose, particle.weight,
                    particle.weightSum, particle.gweight, particle.previousIndex, particle.node);
            List<Object> emit = new ArrayList<Object>();
            emit.add(particleValue);
            outputCollector.emit(emit);
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new backtype.storm.tuple.Fields(Constants.Fields.SENSOR_ID_FIELD,
                Constants.Fields.TIME_FIELD,
                Constants.Fields.PARTICLE_VALUE));
    }
}
