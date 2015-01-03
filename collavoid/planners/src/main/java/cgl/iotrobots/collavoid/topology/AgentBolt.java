package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.planners.*;
import cgl.iotrobots.collavoid.commons.rabbitmq.Message;
import cgl.iotrobots.collavoid.commons.rabbitmq.RabbitMQSender;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

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
    //    private String routingKey;
    private String exchange;

    @Override
    public void execute(Tuple tuple) {
        if (agent.Name.equals("")) {
            agent.Name = (String) tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            agent.updateBaseFrame();
//            routingKey=Constant_msg.RMQ_ROUTINGKEY_PREFIX+Constant_msg.KEY_POSE_SHARE;
            poseShareSender = new RabbitMQSender(Constant_msg.RMQ_URL, exchange, true);
            try {
                poseShareSender.open();
            } catch (Exception e) {
                e.printStackTrace();
            }
            collector.emit(Constant_storm.Streams.FOOTPRINT_OWN_STREAM, new Values(
                    tuple.getValue(0),
                    tuple.getValue(1),
                    agent.footprint_original));
        }
        String streamId = tuple.getSourceStreamId();
        if (streamId.equals(Constant_storm.Streams.PUBLISHME_STREAM)) {
            updateAgentToPub(tuple);
            Message msg;
            try {
                msg = new Message(poseShareMsg_.toJSON(), new HashMap<String, Object>());
                poseShareSender.send(msg, "");
            } catch (IOException e) {
                e.printStackTrace();
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else if (streamId.equals(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM)) {
            updateAgentToCalVel(tuple);
            collector.emit(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM, new Values(
                    tuple.getValue(0),
                    tuple.getValue(1),
                    agent));
        }
    }

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
        exchange = Constant_msg.KEY_POSE_SHARE;
        agent = new Agent();

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
        upDateAgentState(agent);
        updateAllNeighbors(agent);
        computeMinDistToAll(agent);
    }

    void updateAllNeighbors(Agent agent) {
        for (Agent agent1 : agent.AgentNeighbors) {
            upDateAgentState(agent1);
        }
        agent.AgentNeighbors.sort(new Comparators.NeighborDistComparator(
                agent.position.getPos()));
    }

    private void upDateAgentState(Agent agt) {
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
            y_dif = time_dif * agt.getBaseOdom().getTwist().getLinear().getY() * Math.sin(yaw + th_dif / 2.0);
        }
        theta = yaw + th_dif;
        x = agt.getBaseOdom().getPose().getPosition().getX() + x_dif;
        y = agt.getBaseOdom().getPose().getPosition().getY() + y_dif;
        agt.setPosition(new Position(x, y, theta));


        //minkowski footprint is in robot frame, in velocity space only need orientation.
        agt.setFootPrint_rotated(Methods_Planners.rotateFootprint(
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
