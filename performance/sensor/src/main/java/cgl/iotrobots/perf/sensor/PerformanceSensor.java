package cgl.iotrobots.perf.sensor;

import cgl.iotcloud.core.*;
import cgl.iotcloud.core.msg.MessageContext;
import cgl.iotcloud.core.sensorsite.SiteContext;
import cgl.iotcloud.core.transport.Channel;
import cgl.iotcloud.core.transport.Direction;
import cgl.iotcloud.core.transport.TransportConstants;
import org.apache.commons.cli.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class PerformanceSensor extends AbstractSensor {
    private static Logger LOG = LoggerFactory.getLogger(PerformanceSensor.class);

    public static final String PERF_SEND_DATA_ROUTING_KEY = "perf_send_data";
    public static final String PERF_SEND_DATA_QUEUE_NAME = "perf_send_data";
    public static final String PERF_RECV_QUEUE_NAME = "perf_recv_data";
    public static final String PERF_RECV_ROUTING_KEY = "perf_recv_data";
    public static final String DATA_SENDER = "data_sender";
    public static final String DATA_RECEIVER = "data_receiver";
    private static final String PERF_EXCHANGE = "perf";


    public static final String MODE_ARG = "mode";
    public static final String TRP_ARG = "trp";
    public static final String DATA_SIZE_ARG = "data";
    public static final String DATA_INTERVAL = "freq";
    public static final String

    private boolean run = true;

    public static void main(String[] args) {
        Map<String, String> properties = getProperties(args);
        SensorSubmitter.submitSensor(properties, new java.io.File(PerformanceSensor.class.getProtectionDomain()
                .getCodeSource().getLocation().getPath()).getName(),
                PerformanceSensor.class.getCanonicalName(), Arrays.asList("local"));
    }

    @Override
    public Configurator getConfigurator(Map map) {
        return new STSensorConfigurator();
    }

    @Override
    public void open(SensorContext context) {
        final Channel sendChannel = context.getChannel(TransportConstants.TRANSPORT_RABBITMQ, DATA_SENDER);
        final Channel receiveChannel = context.getChannel(TransportConstants.TRANSPORT_RABBITMQ, DATA_RECEIVER);

        // startSend(sendChannel, receivingQueue);
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                while (run) {
                    try {
                        Map<String, Object> props = new HashMap<String, Object>();
                        props.put("time", Long.toString(System.currentTimeMillis()));

                        sendChannel.publish(body, props);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        });
        t.start();

        startListen(receiveChannel, new cgl.iotcloud.core.MessageReceiver() {
            @Override
            public void onMessage(Object message) {
                if (message instanceof MessageContext) {
                    LOG.info("Message received " + message.toString());

                } else {
                    LOG.error("Unexpected message");
                }
            }
        });
        LOG.info("Received request for opening sensor: {} with id: {}", context.getSensorID());
    }

    @Override
    public void close() {
        run = false;
        super.close();
    }

    @SuppressWarnings("unchecked")
    private class STSensorConfigurator extends AbstractConfigurator {
        @Override
        public SensorContext configure(SiteContext siteContext, Map conf) {
            String mode = (String) conf.get(MODE_ARG);

            SensorContext context = new SensorContext("turtle_sensor");
            context.addProperty(MODE_ARG, mode);

            Map sendProps = new HashMap();
            sendProps.put("exchange", PERF_EXCHANGE);
            sendProps.put("routingKey", PERF_SEND_DATA_ROUTING_KEY);
            sendProps.put("queueName", PERF_SEND_DATA_QUEUE_NAME);
            Channel sendChannel = createChannel(DATA_SENDER, sendProps, Direction.OUT, 1024);

            Map receiveProps = new HashMap();
            receiveProps.put("queueName", PERF_RECV_QUEUE_NAME);
            receiveProps.put("exchange", PERF_EXCHANGE);
            receiveProps.put("routingKey", PERF_RECV_ROUTING_KEY);
            Channel receiveChannel = createChannel(DATA_RECEIVER, receiveProps, Direction.IN, 1024);

            context.addChannel(TransportConstants.TRANSPORT_RABBITMQ, sendChannel);
            context.addChannel(TransportConstants.TRANSPORT_RABBITMQ, receiveChannel);

            return context;
        }
    }

    private static Map<String, String> getProperties(String []args) {
        Map<String, String> conf = new HashMap<String, String>();
        Options options = new Options();
        options.addOption(MODE_ARG, true, "possible options are (nt, t) nt means without connecting to turtle");
        options.addOption(TRP_ARG, true, "k or r");
        CommandLineParser commandLineParser = new BasicParser();
        try {
            CommandLine cmd = commandLineParser.parse(options, args);
            String mode = cmd.getOptionValue(MODE_ARG);
            String trp = cmd.getOptionValue(TRP_ARG);

            conf.put(MODE_ARG, mode);
            conf.put(TRP_ARG, trp);

            return conf;
        } catch (ParseException e) {
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp("sensor", options );
        }
        return null;
    }
}
