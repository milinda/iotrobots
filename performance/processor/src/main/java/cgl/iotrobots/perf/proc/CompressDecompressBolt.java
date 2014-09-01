package cgl.iotrobots.perf.proc;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.xerial.snappy.Snappy;

import java.io.IOException;
import java.util.Arrays;
import java.util.Map;

public class CompressDecompressBolt extends BaseRichBolt {
    private static Logger LOG = LoggerFactory.getLogger(CompressDecompressBolt.class);

    private OutputCollector outputCollector;

    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        this.outputCollector = outputCollector;
    }

    @Override
    public void execute(Tuple tuple) {
        Object data = tuple.getValueByField(Constants.DATA_FILED);
        Object sensorID = tuple.getValueByField(Constants.SENSOR_ID_FIELD);
        Object time = tuple.getValueByField(Constants.TIME_FIELD);

        if (data instanceof byte []) {
            byte []inData = (byte[]) data;

            try {
                byte []compressed = Snappy.compress(inData);
                byte []decompressed = Snappy.uncompress(compressed);
                outputCollector.emit(Arrays.<Object>asList(decompressed, sensorID, time));
            } catch (IOException e) {
                LOG.error("Failed to compress-decompress using snappy");
            }
        } else {
            LOG.warn("Got un-recognizable data");
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new Fields(Constants.DATA_FILED, Constants.SENSOR_ID_FIELD, Constants.TIME_FIELD));
    }
}
