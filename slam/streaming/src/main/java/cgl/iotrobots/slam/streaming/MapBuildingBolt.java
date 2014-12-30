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
import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.gridfastsalm.Particle;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.streaming.msgs.ParticleValue;
import cgl.iotrobots.slam.streaming.msgs.TransferMap;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import com.esotericsoftware.kryo.Kryo;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class MapBuildingBolt extends BaseRichBolt {
    private MapUpdater mapUpdater;

    private OutputCollector outputCollector;

    private TopologyContext topologyContext;

    private Kryo kryo;

    long lastMessageTime;
    long lastComputationTime;
    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        this.outputCollector = outputCollector;
        this.topologyContext = topologyContext;
        this.kryo = new Kryo();
        // read the configuration of the scanmatcher from topology.xml
        StreamTopologyBuilder streamTopologyBuilder = new StreamTopologyBuilder();
        StreamComponents components = streamTopologyBuilder.buildComponents();
        // use the configuration to create the scanmatcher
        GFSConfiguration cfg = ConfigurationBuilder.getConfiguration(components.getConf());

        this.mapUpdater = ProcessorFactory.createMapBuilder(cfg);
    }

    @Override
    public void execute(Tuple tuple) {
        Object time = tuple.getValueByField(Constants.Fields.TIME_FIELD);
        long t0 = System.currentTimeMillis();
        long currentMessageTime = Long.parseLong(time.toString());
        // if this message came within that window, discard it
        // this will allow us to keep track of the current interval
        if (currentMessageTime < lastComputationTime * 10 + lastMessageTime) {
            outputCollector.ack(tuple);
            return;
        }

        Object sensorId = tuple.getValueByField(TransportConstants.SENSOR_ID);

        Object val = tuple.getValueByField(Constants.Fields.PARTICLE_VALUE_FIELD);
        if (!(val instanceof ParticleValue)) {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }
        ParticleValue particleValue = (ParticleValue) val;

        val = tuple.getValueByField(Constants.Fields.PARTICLE_MAP_FIELD);
        if (!(val instanceof TransferMap)) {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }
        TransferMap transferMap = (TransferMap) val;

        val = tuple.getValueByField(Constants.Fields.LASER_SCAN_FIELD);
        if (!(val instanceof LaserScan)) {
            throw new IllegalArgumentException("The laser scan should be of type RangeReading");
        }
        LaserScan scan = (LaserScan) val;

        Particle p = new Particle();
        Utils.createParticle(particleValue, p);
        GMap m = Utils.createGMap(transferMap);
        p.setMap(m);

        double[] laser_angles = new double[scan.getRanges().size()];
        double theta = scan.getAngle_min();
        for (int i = 0; i < scan.getRanges().size(); i++) {
            if (scan.getAngle_increment() < 0)
                laser_angles[scan.getRanges().size() - i - 1] = theta;
            else
                laser_angles[i] = theta;
            theta += scan.getAngle_increment();
        }

        GFSMap map = mapUpdater.updateMap(p, laser_angles, new DoubleOrientedPoint(0, 0, 0));

        this.outputCollector.ack(tuple);

        byte []body = Utils.serialize(kryo, map);
        List<Object> emit = new ArrayList<Object>();
        emit.add(body);
        emit.add(sensorId);
        emit.add(time);

        outputCollector.emit(emit);
        lastComputationTime = System.currentTimeMillis() - t0;
        lastMessageTime = Long.parseLong(time.toString());
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new backtype.storm.tuple.Fields("body", Constants.Fields.SENSOR_ID_FIELD,
                Constants.Fields.TIME_FIELD
                ));
    }
}
