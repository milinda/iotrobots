package cgl.iotrobots.collavoid.topology;

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
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import com.esotericsoftware.kryo.Kryo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AgentBolt extends BaseRichBolt {
    private OutputCollector collector;
    private Agent agent;
    private PoseShareMsg_ poseShareMsg_ = new PoseShareMsg_();
    private RabbitMQSender poseShareSender;
    private String exchange;
    private Logger logger = LoggerFactory.getLogger(AgentBolt.class);
    private boolean footprintSent = false;

    @Override
    public void execute(Tuple tuple) {
        if (agent.Name.equals("")) {
            agent.Name = (String) tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            agent.updateBaseFrame();
            poseShareSender = new RabbitMQSender(Constant_msg.RMQ_URL, exchange);
            try {
                poseShareSender.open(Constant_msg.TYPE_EXCHANGE_FANOUT);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        Object tupleObj = tuple.getValueByField(Constant_storm.FIELDS.FOOTPRINT_MINKOWSK_FIELD);
        if (tupleObj != null) {
            agent.lock.lock();
            try {
                agent.setFootprint_minkowski((List<Vector2>) tupleObj);
            } finally {
                agent.lock.unlock();
            }
        }

        if (!footprintSent) {
            collector.emit(Constant_storm.Streams.FOOTPRINT_OWN_STREAM,
                    new Values(
                            tuple.getValueByField(Constant_storm.FIELDS.TIME_FIELD),
                            tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                            // this will not change and will not be modified, so just send original variable not a copy
                            agent.footprint_original
                    )
            );
            footprintSent = true;
        }

        String streamId = tuple.getSourceStreamId();
//        if (tuple.getSourceComponent().equals(Constant_storm.Components.POSE_ARRAY_COMPONENT)) {
//            PoseArray_ poseArray_ = (PoseArray_) tuple.getValueByField(Constant_storm.FIELDS.POSE_ARRAY_FIELD);
//            getMinkowskiFootprint(poseArray_);
//        } else
        if (streamId.equals(Constant_storm.Streams.PUBLISHME_STREAM)) {
            if (updateAgentToPub(tuple)) {
                Message msg;
                try {
                    msg = new Message(Methods_RMQ.serialize(poseShareMsg_), new HashMap<String, Object>());
                    poseShareSender.send(msg, "");
                } catch (IOException e) {
                    e.printStackTrace();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        } else if (streamId.equals(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM)) {
            if (updateAgentToCalVel(tuple)) {
                collector.emit(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM,
                        new Values(
                                tuple.getValue(0),
                                tuple.getValue(1),
                                Utils.serialize(agent)));
            }
        }

        collector.ack(tuple);
    }

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
        exchange = Constant_msg.KEY_POSE_SHARE;
        agent = new Agent("");
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
                agent.base_odom_ == null)
            return false;

        if (tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD) != null) {
            agent.base_odom_ = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        }

        poseShareMsg_.getHeader().setFrameId(agent.getBase_frame_());
        poseShareMsg_.getHeader().setStamp(System.currentTimeMillis());
        poseShareMsg_.setPose(agent.getBaseOdom().getPose());
        poseShareMsg_.setTwist(agent.getBaseOdom().getTwist());
        poseShareMsg_.setControlled(agent.getController());
        poseShareMsg_.setHoloRobot(agent.getHoloRobot());
        poseShareMsg_.setRadius(agent.getRadius() + agent.getCur_loc_unc_radius_());
        poseShareMsg_.setName(agent.getName());
        poseShareMsg_.setControlPeriod(agent.controlPeriod);
        poseShareMsg_.setFootPrint_Minkowski(agent.getFootprint_minkowski());

        return true;
    }

    private boolean updateAgentToCalVel(Tuple tuple) {
        agent.base_odom_ = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        agent.AgentNeighbors = (List<Neighbor>) tuple.getValueByField(Constant_storm.FIELDS.NEIGHBORS_FIELD);
        agent.obstacles_from_laser_ = (List<Obstacle>) tuple.getValueByField(Constant_storm.FIELDS.OBSTACLE_FIELD);
        agent.prefVelociy = (Vector2) tuple.getValueByField(Constant_storm.FIELDS.PREFERRED_VELOCITY_FIELD);
        agent.last_seen_ = agent.base_odom_.getHeader().getStamp();
        agent.lock.lock();
        try {
            updateAgentState(agent);
            updateAllNeighbors(agent);
            computeMinDistToAll(agent);
        } finally {
            agent.lock.unlock();
        }

        return true;
    }

    void updateAllNeighbors(Agent agent) {
        for (Neighbor agent1 : agent.AgentNeighbors) {
            updateAgentState(agent1);
        }
        agent.AgentNeighbors.sort(new Comparators.NeighborDistComparator(
                agent.position.getPos()));
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
