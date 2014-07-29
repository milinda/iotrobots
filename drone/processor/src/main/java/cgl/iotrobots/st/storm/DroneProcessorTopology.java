/*
 *
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 *
 */

package cgl.iotrobots.st.storm;

import backtype.storm.Config;
import backtype.storm.LocalCluster;
import backtype.storm.StormSubmitter;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.TopologyBuilder;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import com.rabbitmq.client.AMQP;
import com.ss.rabbitmq.*;
import com.ss.rabbitmq.bolt.RabbitMQBolt;
import org.apache.commons.cli.BasicParser;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.Options;
import org.apache.commons.codec.binary.Base64;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class DroneProcessorTopology {
    public static void main(String[] args) throws Exception {
        TopologyBuilder builder = new TopologyBuilder();
        ErrorReporter r = new ErrorReporter() {
            @Override
            public void reportError(Throwable t) {
                t.printStackTrace();
            }
        };

        Options options = new Options();
        options.addOption(Constants.ARGS_URL, true, "URL of the AMQP Broker");
        options.addOption(Constants.ARGS_NAME, true, "Name of the topology");
        options.addOption(Constants.ARGS_LOCAL, false, "Weather we want run locally");

        CommandLineParser commandLineParser = new BasicParser();
        CommandLine cmd = commandLineParser.parse(options, args);
        String url = cmd.getOptionValue(Constants.ARGS_URL);
        String name = cmd.getOptionValue(Constants.ARGS_NAME);
        boolean local = cmd.hasOption(Constants.ARGS_LOCAL);

        //buildAllSeparateTopology(builder, r, url);
        buildDecodeAndTrackingTopology(builder, r, url);

        Config conf = new Config();
        conf.setDebug(false);
        // we are not going to track individual messages, message loss is inherent in the decoder
        // also we cannot replay message because of the decoder
        conf.put(Config.TOPOLOGY_ACKER_EXECUTORS, 0);

        // we are going to deploy on a real cluster
        if (!local) {
            conf.setNumWorkers(5);
            StormSubmitter.submitTopology(name, conf, builder.createTopology());
        } else {
            // deploy on a local cluster
            conf.setMaxTaskParallelism(3);
            LocalCluster cluster = new LocalCluster();
            cluster.submitTopology("drone", conf, builder.createTopology());
            Thread.sleep(1000000);
            cluster.shutdown();
        }
    }

    private static void buildDecodeAndTrackingTopology(TopologyBuilder builder, ErrorReporter r, String url) {
        builder.setSpout(Constants.FRAME_RECEIVE_SPOUT, new RabbitMQSpout(new ReceiveSpoutConfigurator(url), r), 1);
        builder.setBolt(Constants.DECODE_BOLT, new DecodingBolt()).shuffleGrouping(Constants.FRAME_RECEIVE_SPOUT);
        builder.setBolt(Constants.TRACKING_BOLT, new TrackingBolt()).shuffleGrouping(Constants.DECODE_BOLT);
        builder.setBolt(Constants.PLANING_BOLT, new PlanningBolt()).shuffleGrouping(Constants.TRACKING_BOLT);
        builder.setBolt(Constants.SEND_COMMAND_BOLT, new RabbitMQBolt(new OutputBoltConfigurator(url), r)).shuffleGrouping(Constants.PLANING_BOLT);
    }

    private static void buildAllSeparateTopology(TopologyBuilder builder, ErrorReporter r, String url) {
        builder.setSpout(Constants.FRAME_RECEIVE_SPOUT, new RabbitMQSpout(new ReceiveSpoutConfigurator(url, true), r), 1);
        builder.setBolt(Constants.DECODE_AND_TRACKING_BOLT, new DecodeTrackingBolt()).shuffleGrouping(Constants.FRAME_RECEIVE_SPOUT);
        builder.setBolt(Constants.PLANING_BOLT, new PlanningBolt()).shuffleGrouping(Constants.DECODE_AND_TRACKING_BOLT);
        builder.setBolt(Constants.SEND_COMMAND_BOLT, new RabbitMQBolt(new OutputBoltConfigurator(url), r)).shuffleGrouping(Constants.PLANING_BOLT);
    }

    private static class ReceiveSpoutConfigurator implements RabbitMQConfigurator {
        private String url = "amqp://localhost:5672";

        private boolean base64Encode;

        private ReceiveSpoutConfigurator(String url) {
            this.url = url;
        }

        private ReceiveSpoutConfigurator(String url, boolean base64Encode) {
            this.url = url;
            this.base64Encode = base64Encode;
        }

        @Override
        public String getURL() {
            return url;
        }

        @Override
        public boolean isAutoAcking() {
            return true;
        }

        @Override
        public int getPrefetchCount() {
            return 1024;
        }

        @Override
        public boolean isReQueueOnFail() {
            return false;
        }

        @Override
        public String getConsumerTag() {
            return "sender";
        }

        @Override
        public List<RabbitMQDestination> getQueueName() {
            List<RabbitMQDestination> list = new ArrayList<RabbitMQDestination>();
            list.add(new RabbitMQDestination("local-1.storm_drone_frame", "storm_drone", "storm_drone_frame"));
            return list;
        }

        @Override
        public MessageBuilder getMessageBuilder() {
            return new ReceiveBuilder(base64Encode);
        }

        @Override
        public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
            outputFieldsDeclarer.declare(new Fields(Constants.FRAME_FIELD, Constants.TIME_FIELD));
        }

        @Override
        public int queueSize() {
            return 1024;
        }

        @Override
        public RabbitMQDestinationSelector getDestinationSelector() {
            return null;
        }
    }

    private static class OutputBoltConfigurator implements RabbitMQConfigurator {
        private String url = "amqp://localhost:5672";

        private OutputBoltConfigurator(String url) {
            this.url = url;
        }

        @Override
        public String getURL() {
            return url;
        }

        @Override
        public boolean isAutoAcking() {
            return true;
        }

        @Override
        public int getPrefetchCount() {
            return 1024;
        }

        @Override
        public boolean isReQueueOnFail() {
            return false;
        }

        @Override
        public String getConsumerTag() {
            return "control";
        }

        @Override
        public List<RabbitMQDestination> getQueueName() {
            List<RabbitMQDestination> list = new ArrayList<RabbitMQDestination>();
            list.add(new RabbitMQDestination("local-1.storm_control", "storm_drone", "storm_control"));
            return list;
        }

        @Override
        public MessageBuilder getMessageBuilder() {
            return new ReceiveBuilder();
        }

        @Override
        public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
            outputFieldsDeclarer.declare(new Fields(Constants.CONTROL_FIELD));
        }

        @Override
        public int queueSize() {
            return 1024;
        }

        @Override
        public RabbitMQDestinationSelector getDestinationSelector() {
            return new RabbitMQDestinationSelector() {
                @Override
                public String select(Tuple tuple) {
                    return "local-1.storm_control";
                }
            };
        }
    }

    private static class ReceiveBuilder implements MessageBuilder {
        private boolean base64Encode = false;

        private ReceiveBuilder(boolean base64Encode) {
            this.base64Encode = base64Encode;
        }

        private ReceiveBuilder() {
        }

        @Override
        public List<Object> deSerialize(RabbitMQMessage message) {
            Object time = null;
            AMQP.BasicProperties properties = message.getProperties();
            if (properties != null && properties.getHeaders() != null) {
                time = properties.getHeaders().get("time");
            }

            byte []body = message.getBody();
            List<Object> tuples = new ArrayList<Object>();
            if (!base64Encode) {
                tuples.add(body);
            } else {
                tuples.add(Base64.encodeBase64String(body));
            }

            if (time != null) {
                tuples.add(time.toString());
            }

            return tuples;
        }

        @Override
        public RabbitMQMessage serialize(Tuple tuple) {
            String time = (String) tuple.getValueByField(Constants.TIME_FIELD);
            Map<String, Object> props = new HashMap<String, Object>();
            props.put(Constants.TIME_FIELD, time);

            return new RabbitMQMessage(null, null, null,
                    new AMQP.BasicProperties.Builder().headers(props).build(),
                    tuple.getValue(0).toString().getBytes());
        }
    }

}
