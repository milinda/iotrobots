package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.Obstacle;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.ConnectionFactory;
import io.latent.storm.rabbitmq.Declarator;
import io.latent.storm.rabbitmq.Message;
import io.latent.storm.rabbitmq.RabbitMQProducer;
import io.latent.storm.rabbitmq.config.ConnectionConfig;
import io.latent.storm.rabbitmq.config.ConsumerConfig;
import io.latent.storm.rabbitmq.config.ConsumerConfigBuilder;


import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AgentBolt extends BaseRichBolt {
    private OutputCollector collector;
    private Agent agent;
    private PoseShareMsg_ poseShareMsg_ = new PoseShareMsg_();
    private RabbitMQProducer poseShareSender;

    @Override
    public void execute(Tuple tuple) {
        if (agent.Name.equals("")) {
            agent.Name = (String) tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            agent.updateBaseFrame();
        }
        String streamId = tuple.getSourceStreamId();
        if (streamId.equals(Constant_storm.Streams.PUBLISHME_STREAM)) {
            updateAgentToPub(tuple);
            Message msg = new Message(Serializers.serialize(poseShareMsg_));
            poseShareSender.send(msg);
        } else if (streamId.equals(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM)) {
            updateAgentToCalVel(tuple);
            collector.emit(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM, new Values(agent));
        }
    }

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        ConnectionConfig connectionConfig = new ConnectionConfig(
                "localhost",
                5672,
                "guest",
                "guest",
                ConnectionFactory.DEFAULT_VHOST,
                10); // host, port, username, password, virtualHost, heartBeat
        ConsumerConfig spoutConfig = new ConsumerConfigBuilder().connection(connectionConfig)
                .queue("")
                .prefetch(200)
                .requeueOnFail()
                .build();
        poseShareSender = new RabbitMQProducer(new StormDeclarator(
                Constant_msg.KEY_POSE_SHARE,
                Constant_msg.TYPE_EXCHANGE_FANOUT));
        poseShareSender.open(spoutConfig.asMap());
        collector = outputCollector;
        agent = new Agent();
        collector.emit(Constant_storm.Streams.FOOTPRINT_OWN_STREAM, new Values(agent.footprint_original));
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.PUBLISHME_STREAM,
                new Fields(Constant_storm.FIELDS.POSE_SHARE_FIELD));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                new Fields(Constant_storm.FIELDS.AGENT_FIELD));
    }

    private void updateAgentToPub(Tuple tuple) {
        agent.base_odom_ = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        agent.footprint_minkowski = (List<Vector2>) tuple.getValueByField(Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD);

        poseShareMsg_.getHeader().setFrameId(agent.getBase_frame_());
        poseShareMsg_.getHeader().setStamp(System.currentTimeMillis());
        poseShareMsg_.setPose(agent.getBaseOdom().getPose());
        poseShareMsg_.setTwist(agent.getBaseOdom().getTwist());
        poseShareMsg_.setControlled(agent.getController());
        poseShareMsg_.setHoloRobot(agent.getHoloRobot());
        poseShareMsg_.setRadius(agent.getRadius() + agent.getCur_loc_unc_radius_());
        poseShareMsg_.setRobotId(agent.getName());

        List<Vector3d_> footprint = new ArrayList<Vector3d_>();
        for (Vector2 vector2 : agent.getFootprint_minkowski()) {
            Vector3d_ vector3d_ = new Vector3d_(vector2.getX(), vector2.getY(), 0);
            footprint.add(vector3d_);
        }
        poseShareMsg_.setFootPrint_Minkowski(footprint);
    }

    private void updateAgentToCalVel(Tuple tuple) {
        agent.base_odom_ = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        agent.AgentNeighbors = (List<Agent>) tuple.getValueByField(Constant_storm.FIELDS.NEIGHBORS_FIELD);
        agent.footprint_minkowski = (List<Vector2>) tuple.getValueByField(Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD);
        agent.obstacles_from_laser_ = (List<Obstacle>) tuple.getValueByField(Constant_storm.FIELDS.OBSTACLE_FIELD);
        agent.prefVelociy = (Vector2) tuple.getValueByField(Constant_storm.FIELDS.PREFERRED_VELOCITY_FIELD);
    }

    public static class StormDeclarator implements Declarator {
        private final String exchange;
        private final String exType;

        public StormDeclarator(String exchange, String exType) {
            this.exchange = exchange;
            this.exType = exType;
        }


        @Override
        public void execute(Channel channel) {
            // you're given a RabbitMQ Channel so you're free to wire up your exchange/queue bindings as you see fit
            try {
                Map<String, Object> args = new HashMap<String, Object>();
                channel.exchangeDeclare(exchange, exType, true);
            } catch (IOException e) {
                throw new RuntimeException("Error executing rabbitmq declarations.", e);
            }
        }
    }
}
