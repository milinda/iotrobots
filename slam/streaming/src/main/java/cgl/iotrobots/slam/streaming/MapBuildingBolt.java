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
import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.sensor.RangeSensor;
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

        double []plainReading = new double[scan.getRanges().size()];
        for (int i = 0; i < scan.getRanges().size(); i++) {
            plainReading[i] = reading.get(i);
        }

        Particle p = new Particle();
        Utils.createParticle(particleValue, p);
        GMap m = Utils.createGMap(transferMap);
        p.setMap(m);

        GFSMap map = mapUpdater.updateMap(p, plainReading, new DoubleOrientedPoint(0, 0, 0));

        this.outputCollector.ack(tuple);

        byte []body = Utils.serialize(kryo, map);
        List<Object> emit = new ArrayList<Object>();
        emit.add(body);
        emit.add(sensorId);
        emit.add(time);

        outputCollector.emit(emit);
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new backtype.storm.tuple.Fields("body", Constants.Fields.SENSOR_ID_FIELD,
                Constants.Fields.TIME_FIELD
                ));
    }
}
