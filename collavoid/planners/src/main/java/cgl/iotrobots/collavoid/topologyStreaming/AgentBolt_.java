package cgl.iotrobots.collavoid.topologyStreaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.planners.*;
import cgl.iotrobots.collavoid.commons.rabbitmq.Message;
import cgl.iotrobots.collavoid.commons.rabbitmq.RabbitMQSender;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.PoseShareMsg_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AgentBolt_ extends BaseRichBolt {
    private OutputCollector collector;
    private Logger logger = LoggerFactory.getLogger(AgentBolt.class);

    private PoseShareMsg_ poseShareMsg_ = new PoseShareMsg_();
    private RabbitMQSender poseShareSender;
    private String id;
    private Map<String, AgentBoltContext> contexts = new HashMap<String, AgentBoltContext>();
    private AgentBoltContext currentContext;

    private class AgentBoltContext {
        private Agent agent;
        private boolean footprintSent = false;

        public AgentBoltContext(String id) {
            agent = new Agent(id);

        }
    }

    @Override
    public void execute(Tuple tuple) {
        id = (String) tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
        if (contexts.get(id) == null) {
            contexts.put(id, new AgentBoltContext(id));
        }
        currentContext = contexts.get(id);

        Object tupleObj = tuple.getValueByField(Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD);
        if (tupleObj != null) {
            currentContext.agent.lock.lock();
            try {
                currentContext.agent.setFootprint_minkowski((List<Vector2>) tupleObj);
            } finally {
                currentContext.agent.lock.unlock();
            }
        }

        String streamId = tuple.getSourceStreamId();

        if (streamId.equals(Constant_storm.Streams.PUBLISHME_STREAM)) {
            if (updateAgentToPub(tuple)) {
                collector.emit(new Values(tuple.getValue(0),tuple.getValue(1),Utils.serialize(poseShareMsg_)));
//                Message msg;
//                try {
//                    msg = new Message(Methods_RMQ.serialize(poseShareMsg_), new HashMap<String, Object>());
//                    poseShareSender.send(msg, "");
//                } catch (IOException e) {
//                    e.printStackTrace();
//                } catch (Exception e) {
//                    e.printStackTrace();
//                }
            }
        } else if (streamId.equals(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM)) {
            if (updateAgentToCalVel(tuple)) {
                collector.emit(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                        new Values(
                                tuple.getValue(0),
                                tuple.getValue(1),
                                Utils.serialize(currentContext.agent)));
            }
        }
        // what about reset???
        if (!currentContext.footprintSent) {
            collector.emit(Constant_storm.Streams.FOOTPRINT_OWN_STREAM,
                    new Values(
                            tuple.getValueByField(Constant_storm.FIELDS.TIME_FIELD),
                            tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                            // this will not change and will not be modified, so just send original variable not a copy
                            currentContext.agent.footprint_original
                    )
            );
            currentContext.footprintSent = true;
        }

        collector.ack(tuple);
    }

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
//        poseShareSender = new RabbitMQSender(Constant_msg.RMQ_URL, Constant_msg.KEY_POSE_SHARE);
//        try {
//            poseShareSender.open(Constant_msg.TYPE_EXCHANGE_FANOUT);
//        } catch (Exception e) {
//            e.printStackTrace();
//        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                new Fields(
                        Constant_storm.FIELDS.TIME_FIELD,
                        Constant_storm.FIELDS.SENSOR_ID_FIELD,
                        Constant_storm.FIELDS.AGENT_FIELD));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.FOOTPRINT_OWN_STREAM,
                new Fields(
                        Constant_storm.FIELDS.TIME_FIELD,
                        Constant_storm.FIELDS.SENSOR_ID_FIELD,
                        Constant_storm.FIELDS.FOOTPRINT_OWN_FIELD));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.PUBLISHME_STREAM,
                new Fields(
                        Constant_storm.FIELDS.TIME_FIELD,
                        Constant_storm.FIELDS.SENSOR_ID_FIELD,
                        Constant_storm.FIELDS.POSE_SHARE_FIELD
                )
        );

    }


//
//    private void getMinkowskiFootprint(PoseArray_ poseArray_) {
//        // in robot base frame do not need transform
//        double x, y;
//        List<Vector2> localization_footprint = new ArrayList<Vector2>();
//
//        // select valid localization (need to replace)
//        for (int i = 0; i < poseArray_.getPoses().size(); i++) {
//            x = poseArray_.getPoses().get(i).getPosition().getX();
//            y = poseArray_.getPoses().get(i).getPosition().getY();
//            Vector2 p = new Vector2(x, y);
//            if (p.VectorLength() > 0.1)
//                continue;
//            localization_footprint.add(p);
//        }
//        agent.lock.lock();
//        try {
//            agent.setFootprint_minkowski(Methods_Planners.minkowskiSumConvexHull(
//                    localization_footprint,
//                    agent.footprint_original));
//        }finally {
//            agent.lock.unlock();
//        }
//
//    }

    private boolean updateAgentToPub(Tuple tuple) {
        if (tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD) == null &&
                currentContext.agent.base_odom_ == null)
            return false;

        if (tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD) != null) {
            currentContext.agent.base_odom_ = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        }

        poseShareMsg_.getHeader().setFrameId(currentContext.agent.getBase_frame_());
        poseShareMsg_.getHeader().setStamp(System.currentTimeMillis());
        poseShareMsg_.setPose(currentContext.agent.getBaseOdom().getPose());
        poseShareMsg_.setTwist(currentContext.agent.getBaseOdom().getTwist());
        poseShareMsg_.setControlled(currentContext.agent.getController());
        poseShareMsg_.setHoloRobot(currentContext.agent.getHoloRobot());
        poseShareMsg_.setRadius(currentContext.agent.getRadius() + currentContext.agent.getCur_loc_unc_radius_());
        poseShareMsg_.setName(currentContext.agent.getName());
        poseShareMsg_.setControlPeriod(currentContext.agent.controlPeriod);
        poseShareMsg_.setFootPrint_Minkowski(currentContext.agent.getFootprint_minkowski());

        return true;
    }

    private boolean updateAgentToCalVel(Tuple tuple) {
        currentContext.agent.base_odom_ = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        currentContext.agent.AgentNeighbors = (List<Neighbor>) tuple.getValueByField(Constant_storm.FIELDS.NEIGHBORS_FIELD);
        currentContext.agent.obstacles_from_laser_ = (List<Obstacle>) tuple.getValueByField(Constant_storm.FIELDS.OBSTACLE_FIELD);
        currentContext.agent.prefVelociy = (Vector2) tuple.getValueByField(Constant_storm.FIELDS.PREFERRED_VELOCITY_FIELD);
        currentContext.agent.last_seen_ = currentContext.agent.base_odom_.getHeader().getStamp();
        currentContext.agent.lock.lock();
        try {
            updateAgentState(currentContext.agent);
            updateAllNeighbors(currentContext.agent);
            computeMinDistToAll(currentContext.agent);
        } finally {
            currentContext.agent.lock.unlock();
        }

        return true;
    }

    void updateAllNeighbors(Agent agent) {
        for (Neighbor agent1 : agent.AgentNeighbors) {
            updateAgentState(agent1);
        }
//        agent.AgentNeighbors.sort(new Comparators.NeighborDistComparator(
//                agent.position.getPos()));
        Collections.sort(agent.AgentNeighbors, new Comparators.NeighborDistComparator(agent.position.getPos()));
    }

    private void updateAgentState(Neighbor agt) {
        double time_dif;
        if (agt.getLastSeen() == 0)
            time_dif = 0;
        else
            time_dif = System.currentTimeMillis() - agt.getLastSeen();
        time_dif = time_dif / 1000.0;

        double yaw, x_dif, y_dif, th_dif, x, y, theta;
        Vector2 pt = new Vector2(0.0, 0.0);

        //update position
        yaw = Methods_Planners.getYaw(agt.getBaseOdom().getPose().getOrientation());
        th_dif = time_dif * agt.getBaseOdom().getTwist().getAngular().getZ();
        if (agt.getHoloRobot()) {
            x_dif = time_dif * agt.getBaseOdom().getTwist().getLinear().getX();
            y_dif = time_dif * agt.getBaseOdom().getTwist().getLinear().getY();
        } else {
            x_dif = time_dif * agt.getBaseOdom().getTwist().getLinear().getX() * Math.cos(yaw + th_dif / 2.0);
            y_dif = time_dif * agt.getBaseOdom().getTwist().getLinear().getX() * Math.sin(yaw + th_dif / 2.0);
        }
        theta = yaw + th_dif;
        x = agt.getBaseOdom().getPose().getPosition().getX() + x_dif;
        y = agt.getBaseOdom().getPose().getPosition().getY() + y_dif;
        agt.setPosition(new Position(x, y, theta));


        //minkowski footprint is in robot frame, in velocity space only need orientation.
        agt.setFootprint_rotated(Methods_Planners.rotateFootprint(
                agt.getFootprint_minkowski(),
                agt.getPosition().getHeading()));


        //update velocity
        if (agt.getHoloRobot()) {
            x = agt.getBaseOdom().getTwist().getLinear().getX();
            y = agt.getBaseOdom().getTwist().getLinear().getY();
            pt.setX(x);
            pt.setY(y);
            agt.setVelocity(Vector2.rotateVectorByAngle(pt, (yaw + th_dif)));
        } else {//?????????????????????????????????????????????????????????????
            double dif_x, dif_y, dif_ang;
            dif_ang = agt.getControlPeriod() * agt.getBaseOdom().getTwist().getAngular().getZ();
            //in robot frame differential robot has only x velocity
            pt.setX(agt.getBaseOdom().getTwist().getLinear().getX() * Math.cos(dif_ang / 2.0));
            pt.setY(agt.getBaseOdom().getTwist().getLinear().getX() * Math.sin(dif_ang / 2.0));
            // in global frame, velocity need no translation
            agt.setVelocity(Vector2.rotateVectorByAngle(pt, (yaw + th_dif)));
        }
    }

    private void computeMinDistToAll(Agent agent) {
        double min_dist_neigh = Double.MAX_VALUE;
        double min_dist_obstacle = Double.MAX_VALUE;
        //neighbors have already been sorted according to their dist to me
        if (agent.AgentNeighbors.size() > 0)
            min_dist_neigh = Vector2.abs(Vector2.minus(
                            agent.AgentNeighbors.get(0).position.getPos(),
                            agent.position.getPos())
            );

        double threshold = Math.pow((Vector2.abs(agent.velocity) + 4.0 * agent.footprint_radius_), 2);
        for (Obstacle obstacle : agent.obstacles_from_laser_) {
            double dist = Methods_Planners.distSqPointLineSegment(
                    obstacle.getBegin(),
                    obstacle.getEnd(),
                    agent.position.getPos());
            obstacle.setDistToAgent(dist);
            if (dist < threshold) {
                if (dist < min_dist_obstacle) {
                    min_dist_obstacle = dist;
                }
            }

        }
        agent.min_dist = Math.min(min_dist_neigh, min_dist_obstacle);
    }

}
