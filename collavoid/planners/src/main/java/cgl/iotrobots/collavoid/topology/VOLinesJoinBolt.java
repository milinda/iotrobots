package cgl.iotrobots.collavoid.topology;

import backtype.storm.Config;
import backtype.storm.generated.GlobalStreamId;
import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import backtype.storm.utils.RotatingMap;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.Line;
import cgl.iotrobots.collavoid.commons.planners.VO;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import cgl.iotrobots.collavoid.commons.storm.Methods_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class VOLinesJoinBolt extends BaseRichBolt {
    private Logger logger = LoggerFactory.getLogger(VOLinesJoinBolt.class);
    OutputCollector _collector;
    Fields _idFields;
    Fields _joinFields;
    int _numSources;
    int _timeOut = 10;
    RotatingMap<List<Object>, Map<GlobalStreamId, Tuple>> _pending;
    Map<String, GlobalStreamId> _fieldLocations;
    List<Line> orcalines = new ArrayList<Line>();
    List<VO> vos = new ArrayList<VO>();
    Agent agent;

    public VOLinesJoinBolt(Fields outFields) {
        _joinFields = outFields;
    }

    @Override
    public void prepare(Map conf, TopologyContext context, OutputCollector collector) {
        _fieldLocations = new HashMap<String, GlobalStreamId>();
        _collector = collector;
        _pending = new RotatingMap<List<Object>, Map<GlobalStreamId, Tuple>>(4, new ExpireCallback());
        _numSources = context.getThisSources().size();

        Set<String> idFields = null;
        for (GlobalStreamId source : context.getThisSources().keySet()) {
            Fields fields = context.getComponentOutputFields(source.get_componentId(), source.get_streamId());
            Set<String> setFields = new HashSet<String>(fields.toList());
            if (idFields == null)
                idFields = setFields;
            else
                idFields.retainAll(setFields);
            for (String outfield : _joinFields) {
                for (String sourcefield : fields) {
                    if (outfield.equals(sourcefield)) {
                        _fieldLocations.put(outfield, source);
                    }
                }
            }
        }
        _idFields = new Fields(new ArrayList<String>(idFields));

        if (_fieldLocations.size() != _joinFields.size()) {
            throw new RuntimeException("Cannot find all outfields among sources");
        }
    }

    @Override
    public void execute(Tuple tuple) {
        //if time is up, rotate the pending map
        if (Methods_storm.isTickTuple(tuple)) {
            _pending.rotate();
            return;
        }
        List<Object> id = tuple.select(_idFields);//use rest fields as id
        GlobalStreamId streamId = new GlobalStreamId(tuple.getSourceComponent(), tuple.getSourceStreamId());
        if (!_pending.containsKey(id)) {
            _pending.put(id, new HashMap<GlobalStreamId, Tuple>());
        }
        Map<GlobalStreamId, Tuple> parts = _pending.get(id);
        if (parts.containsKey(streamId))
            throw new RuntimeException("Received same side of single join twice");
        parts.put(streamId, tuple);
        if (parts.size() == _numSources) {
            _pending.remove(id);
            List<Object> joinResult = new ArrayList<Object>();
            for (String outField : _joinFields) {
                GlobalStreamId loc = _fieldLocations.get(outField);
                // results are in _joinFields sequence
                if (outField.equals(Constant_storm.FIELDS.AGENT_FIELD))
                    joinResult.add(Utils.deserialize(parts.get(loc).getBinaryByField(outField)));
                else
                    joinResult.add(parts.get(loc).getValueByField(outField));
            }
            mergeLinesAndVOs(joinResult);
            // do not need to copy the agent as new agent will not be sent until this computation complete
            _collector.emit(new Values(tuple.getValue(0), tuple.getValue(1), Utils.serialize(agent)));

            for (Tuple part : parts.values()) {
                _collector.ack(part);
            }
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.AGENT_FIELD
        ));
    }

    @Override
    public Map<String, Object> getComponentConfiguration() {
        Map<String, Object> conf = new HashMap<String, Object>();
        conf.put(Config.TOPOLOGY_TICK_TUPLE_FREQ_SECS, _timeOut);
        return conf;
    }

    private class ExpireCallback implements RotatingMap.ExpiredCallback<List<Object>, Map<GlobalStreamId, Tuple>> {
        @Override
        public void expire(List<Object> id, Map<GlobalStreamId, Tuple> tuples) {
            for (Tuple tuple : tuples.values()) {
                logger.error("Delete expired tuple: {" + tuple.toString() + "}");
                _collector.fail(tuple);
            }
        }
    }

    private void mergeLinesAndVOs(List<Object> joinResult) {
        orcalines.clear();
        vos.clear();
        agent = (Agent) joinResult.get(0);
        for (int i = 1; i < 4; i++) {
            orcalines.addAll((List<Line>) joinResult.get(i));
        }

        for (int i = 4; i < 6; i++) {
            vos.addAll((List<VO>) joinResult.get(i));
        }

        agent.addOrcaLines = orcalines;
        agent.voAgents = vos;
    }
}
