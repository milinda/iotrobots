package cgl.iotrobots.slam.streaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.slam.streaming.msgs.Ready;
import cgl.iotrobots.slam.streaming.msgs.Trace;
import cgl.iotrobots.utils.rabbitmq.*;
import cgl.sensorstream.core.StreamComponents;
import cgl.sensorstream.core.StreamTopologyBuilder;
import com.esotericsoftware.kryo.Kryo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class DispatcherBolt extends BaseRichBolt {
    private static Logger LOG = LoggerFactory.getLogger(DispatcherBolt.class);

    // output collector
    private OutputCollector outputCollector;

    // topology context
    private TopologyContext topologyContext;

    // use this receiver to synchronize the
    private RabbitMQReceiver readyReceiver;

    // broker url
    private String brokerURL = "amqp://localhost:5672";

    private Kryo kryo;

    private StreamTopologyBuilder streamTopologyBuilder;
    private StreamComponents components;
    private Map conf;

    private int noOfParallelTasks = 0;

    private Tuple currentTuple;

    private Lock lock = new ReentrantLock();

    private enum State {
        WAITING_FOR_READING,
        WAITING_FOR_READY,
        WAITING_ANY
    }

    private State state = State.WAITING_FOR_READING;

    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declareStream(Constants.Fields.SCAN_STREAM, new Fields(
                Constants.Fields.BODY,
                Constants.Fields.SENSOR_ID_FIELD,
                Constants.Fields.TIME_FIELD,
                Constants.Fields.TRACE_FIELD));
    }

    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        this.conf = stormConf;
        this.topologyContext = context;
        this.outputCollector = collector;
        streamTopologyBuilder = new StreamTopologyBuilder();
        components = streamTopologyBuilder.buildComponents();

        this.brokerURL = (String) components.getConf().get(Constants.RABBITMQ_URL);
        if (conf.get(Constants.ARGS_PARALLEL) != null) {
            this.noOfParallelTasks = ((Long) conf.get(Constants.ARGS_PARALLEL)).intValue();
        } else {
            this.noOfParallelTasks = topologyContext.getComponentTasks(Constants.Topology.SCAN_MATCH_BOLT).size();
        }

        kryo = new Kryo();
        Utils.registerClasses(kryo);

        try {
            this.readyReceiver = new RabbitMQReceiver(brokerURL, Constants.Messages.DIRECT_EXCHANGE);
            this.readyReceiver.listen(new ReadyMessageListener());
        } catch (Exception e) {
            String msg = "Failed to create the receiver";
            LOG.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }

    private List<Ready> readyList = new ArrayList<Ready>();

    private long beginTime;

    private long previousTime;

    private long tempBeginTime;

    @Override
    public void execute(Tuple input) {
        String stream = input.getSourceStreamId();
        if (stream.equals(Constants.Fields.READY_STREAM)) {
            byte readyBytes[] = (byte[]) input.getValueByField(Constants.Fields.READY_FIELD);
            Ready ready = (Ready) Utils.deSerialize(kryo, readyBytes, Ready.class);
            handleReady(ready);
            return;
        }

        lock.lock();
        try {
            tempBeginTime = System.currentTimeMillis();
            if (this.state == State.WAITING_FOR_READING) {
                beginTime = tempBeginTime;
                Trace t = new Trace();
                t.setPd(previousTime);
                outputCollector.emit(Constants.Fields.SCAN_STREAM, createTuple(input, t));
                this.currentTuple = null;
                this.state = State.WAITING_ANY;
                LOG.info("Changing state from READING to ANY");
            } else if (this.state == State.WAITING_ANY) {
                this.currentTuple = input;
                this.state = State.WAITING_FOR_READY;
                LOG.info("Changing state from ANY to READY");
            } else if (this.state == State.WAITING_FOR_READY) {
                this.currentTuple = input;
                LOG.info("Input while in state READY");
            }
        } finally {
            lock.unlock();
        }
    }

    private class ReadyMessageListener implements MessageHandler {
        @Override
        public Map<String, String> getProperties() {
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_QUEUE, Constants.Messages.READY_ROUTING_KEY);
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, Constants.Messages.READY_ROUTING_KEY);
            return props;
        }

        @Override
        public void onMessage(Message message) {
            byte []messageBody = message.getBody();
            Ready ready = (Ready) Utils.deSerialize(kryo, messageBody, Ready.class);

            handleReady(ready);
        }
    }

    private void handleReady(Ready ready) {
        lock.lock();
        readyList.add(ready);
        try {
            if (readyList.size() == noOfParallelTasks) {
                previousTime = System.currentTimeMillis() - beginTime;
                if (state == State.WAITING_FOR_READY) {
//                        beginTime = tempBeginTime;
//                        Trace t = new Trace();
//                        t.setPd(previousTime);
//                        List<Object> emit = createTuple(currentTuple, t);
//                        outputCollector.emit(Constants.Fields.SCAN_STREAM, emit);
                    readyList.clear();
                    currentTuple = null;
                    state = State.WAITING_FOR_READING;
                    LOG.info("Changing state from READY to ANY");
                } else if (state == State.WAITING_ANY) {
                    state = State.WAITING_FOR_READING;
                    readyList.clear();
                    LOG.info("Changing state from ANY to READING");
                }
            }
        } finally {
            lock.unlock();
        }
    }

    private List<Object> createTuple(Tuple currentTuple, Trace trace) {
        Object body = currentTuple.getValueByField(Constants.Fields.BODY);
        Object time = currentTuple.getValueByField(Constants.Fields.TIME_FIELD);
        Object sensorId = currentTuple.getValueByField(Constants.Fields.SENSOR_ID_FIELD);

        List<Object> emit = new ArrayList<Object>();
        emit.add(body);
        emit.add(sensorId);
        emit.add(time);
        emit.add(trace);

        return emit;
    }
}
