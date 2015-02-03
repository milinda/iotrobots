package cgl.iotrobots.slam.streaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.slam.core.GFSConfiguration;
import cgl.iotrobots.slam.core.app.GFSMap;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.app.MapUpdater;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.streaming.msgs.ParticleValue;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import com.esotericsoftware.kryo.Kryo;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MapBuildingBolt extends BaseRichBolt {
    private MapUpdater mapUpdater;

    private OutputCollector outputCollector;

    private Kryo kryo;

    long lastMessageTime;
    long lastComputationTime;
    private ExecutorService executor;

    private enum State {
        WAITING_INPUT,
        EXECUTING
    }

    private State stat = State.WAITING_INPUT;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        this.outputCollector = outputCollector;
        this.kryo = new Kryo();
        executor = Executors.newFixedThreadPool(2);
        // read the configuration of the scanmatcher from topology.xml
        StreamTopologyBuilder streamTopologyBuilder = new StreamTopologyBuilder();
        StreamComponents components = streamTopologyBuilder.buildComponents();
        // use the configuration to create the scanmatcher
        GFSConfiguration cfg = ConfigurationBuilder.getConfiguration(components.getConf());

        this.mapUpdater = ProcessorFactory.createMapBuilder(cfg);

        this.stat = State.WAITING_INPUT;
    }

    @Override
    public void execute(Tuple tuple) {
        this.outputCollector.ack(tuple);

        if (stat == State.WAITING_INPUT) {
            stat = State.EXECUTING;
            Worker worker = new Worker(tuple);
            executor.submit(worker);
        } else {
            return;
        }

//        Object time = tuple.getValueByField(Constants.Fields.TIME_FIELD);
//        long t0 = System.currentTimeMillis();
//        long currentMessageTime = Long.parseLong(time.toString());
//        // if this message came within that window, discard it
//        // this will allow us to keep track of the current interval
//        if (currentMessageTime < lastComputationTime * 2 + lastMessageTime) {
//            outputCollector.ack(tuple);
//            return;
//        }
//
//        Object sensorId = tuple.getValueByField(TransportConstants.SENSOR_ID);
//
//        Object val = tuple.getValueByField(Constants.Fields.PARTICLE_VALUE_FIELD);
//        if (!(val instanceof ParticleValue)) {
//            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
//        }
//        ParticleValue particleValue = (ParticleValue) val;
//
//        val = tuple.getValueByField(Constants.Fields.LASER_SCAN_FIELD);
//        if (!(val instanceof LaserScan)) {
//            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
//        }
//        LaserScan scan = (LaserScan) val;
//
//        Particle p = new Particle();
//        Utils.createParticle(particleValue, p);
//
//        double[] laser_angles = new double[scan.getRanges().size()];
//        double theta = scan.getAngleMin();
//        for (int i = 0; i < scan.getRanges().size(); i++) {
//            if (scan.getAngleIncrement() < 0)
//                laser_angles[scan.getRanges().size() - i - 1] = theta;
//            else
//                laser_angles[i] = theta;
//            theta += scan.getAngleIncrement();
//        }
//
//        GFSMap map = mapUpdater.updateMap(p, laser_angles, new DoubleOrientedPoint(0, 0, 0));
//
//        byte []body = Utils.serialize(kryo, map);
//        List<Object> emit = new ArrayList<Object>();
//        emit.add(body);
//        emit.add(sensorId);
//        emit.add(time);
//
//        outputCollector.emit(emit);
//        lastComputationTime = System.currentTimeMillis() - t0;
//        lastMessageTime = Long.parseLong(time.toString());
    }

    public class Worker implements Runnable {
        Tuple tuple;

        public Worker(Tuple tuple) {
            this.tuple = tuple;
        }

        @Override
        public void run() {
            Object time = tuple.getValueByField(Constants.Fields.TIME_FIELD);
//            long t0 = System.currentTimeMillis();
//            long currentMessageTime = Long.parseLong(time.toString());
            // if this message came within that window, discard it
            // this will allow us to keep track of the current interval
//            if (currentMessageTime < lastComputationTime * 2 + lastMessageTime) {
//                outputCollector.ack(tuple);
//                return;
//            }

            Object sensorId = tuple.getValueByField(TransportConstants.SENSOR_ID);

            Object val = tuple.getValueByField(Constants.Fields.PARTICLE_VALUE_FIELD);
            if (!(val instanceof ParticleValue)) {
                throw new IllegalArgumentException("The laser scan should be of type RangeReading");
            }
            ParticleValue particleValue = (ParticleValue) val;

            val = tuple.getValueByField(Constants.Fields.LASER_SCAN_FIELD);
            if (!(val instanceof LaserScan)) {
                throw new IllegalArgumentException("The laser scan should be of type RangeReading");
            }
            LaserScan scan = (LaserScan) val;

            Particle p = new Particle();
            Utils.createParticle(particleValue, p);

            double[] laser_angles = new double[scan.getRanges().size()];
            double theta = scan.getAngleMin();
            for (int i = 0; i < scan.getRanges().size(); i++) {
                if (scan.getAngleIncrement() < 0)
                    laser_angles[scan.getRanges().size() - i - 1] = theta;
                else
                    laser_angles[i] = theta;
                theta += scan.getAngleIncrement();
            }

            GFSMap map = mapUpdater.updateMap(p, laser_angles, new DoubleOrientedPoint(0, 0, 0));



            byte []body = Utils.serialize(kryo, map);
            List<Object> emit = new ArrayList<Object>();
            emit.add(body);
            emit.add(sensorId);
            emit.add(time);

            outputCollector.emit(emit);
            stat = State.WAITING_INPUT;
//            lastComputationTime = System.currentTimeMillis() - t0;
//            lastMessageTime = Long.parseLong(time.toString());
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new backtype.storm.tuple.Fields("body", Constants.Fields.SENSOR_ID_FIELD,
                Constants.Fields.TIME_FIELD
                ));
    }
}
