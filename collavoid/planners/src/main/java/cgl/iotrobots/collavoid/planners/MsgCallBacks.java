package cgl.iotrobots.collavoid.planners;

import cgl.iotrobots.collavoid.commons.planners.*;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import com.rabbitmq.client.AMQP;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;
import org.jboss.netty.buffer.ChannelBuffer;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;

public class MsgCallBacks {

    private static Logger logger = Logger.getLogger("CallBackLogger");

    public static void bindCallBacks(final Agent agent, Map<String, RMQContext> contexts) throws Exception {
        for (Map.Entry<String, RMQContext> e : contexts.entrySet()) {
            if (e.getKey().equals(Constant.KEY_ODOMETRY))
                bindOdomCallback(agent, e.getValue());
            if (e.getKey().equals(Constant.KEY_SCAN))
                bindScanCallback(agent, e.getValue());
            if (e.getKey().equals(Constant.KEY_PARTICLE_CLOUD))
                bindParticleCloudCallback(agent, e.getValue());
            if (e.getKey().equals(Constant.KEY_POSE_SHARE))
                bindPoseShareCallback(agent, e.getValue());
        }
    }

    private static void bindOdomCallback(final Agent agent, final RMQContext context) {
        try {
            context.CHANNEL.basicConsume(context.QUEUE_NAME, false, context.ROUTING_KEY + "Tag",
                    new DefaultConsumer(context.CHANNEL) {
                        @Override
                        public void handleDelivery(String consumerTag,
                                                   Envelope envelope,
                                                   AMQP.BasicProperties properties,
                                                   byte[] body)
                                throws IOException {
                            long deliveryTag = envelope.getDeliveryTag();
                            Odometry_ odometry_ = JsonConverter.JSONToOdometry_(body);
                            odomCallback(agent, odometry_);
                            context.CHANNEL.basicAck(deliveryTag, false);
                        }
                    });
        } catch (IOException e) {
            String msg = "Error consuming the message";
            throw new RuntimeException(msg, e);
        } catch (Exception e) {
            String msg = "Error connecting to broker";
            throw new RuntimeException(msg, e);
        }
    }

    private static void bindPoseShareCallback(final Agent agent, final RMQContext context) {
        try {

            context.CHANNEL.basicConsume(context.QUEUE_NAME, false, "PoseShare" + "Tag",
                    new DefaultConsumer(context.CHANNEL) {
                        @Override
                        public void handleDelivery(String consumerTag,
                                                   Envelope envelope,
                                                   AMQP.BasicProperties properties,
                                                   byte[] body)
                                throws IOException {
                            long deliveryTag = envelope.getDeliveryTag();
                            PoseShareMsg_ poseShareMsg_ = JsonConverter.JSONToPoseShareMsg_(body);
                            poseShareCallback(agent, poseShareMsg_);
                            context.CHANNEL.basicAck(deliveryTag, false);
                        }
                    });
        } catch (IOException e) {
            String msg = "Error consuming the message";
            throw new RuntimeException(msg, e);
        } catch (Exception e) {
            String msg = "Error connecting to broker";
            throw new RuntimeException(msg, e);
        }
    }

    private static void bindScanCallback(final Agent agent, final RMQContext context) {
        try {

            context.CHANNEL.basicConsume(context.QUEUE_NAME, false, context.ROUTING_KEY + "Tag",
                    new DefaultConsumer(context.CHANNEL) {
                        @Override
                        public void handleDelivery(String consumerTag,
                                                   Envelope envelope,
                                                   AMQP.BasicProperties properties,
                                                   byte[] body)
                                throws IOException {
                            long deliveryTag = envelope.getDeliveryTag();
                            PointCloud2_ pointCloud2_ = JsonConverter.JSONToPointCloud2_(body);
                            scanCallback(agent, pointCloud2_);
                            context.CHANNEL.basicAck(deliveryTag, false);
                        }
                    });
        } catch (IOException e) {
            String msg = "Error consuming the message";
            throw new RuntimeException(msg, e);
        } catch (Exception e) {
            String msg = "Error connecting to broker";
            throw new RuntimeException(msg, e);
        }
    }

    private static void bindParticleCloudCallback(final Agent agent, final RMQContext context) {
        try {

            context.CHANNEL.basicConsume(context.QUEUE_NAME, false, context.ROUTING_KEY + "Tag",
                    new DefaultConsumer(context.CHANNEL) {
                        @Override
                        public void handleDelivery(String consumerTag,
                                                   Envelope envelope,
                                                   AMQP.BasicProperties properties,
                                                   byte[] body)
                                throws IOException {
                            long deliveryTag = envelope.getDeliveryTag();
                            PoseArray_ poseArray_ = JsonConverter.JSONToPoseArray_(body);
                            particleCloudTestCallback(agent, poseArray_);
                            context.CHANNEL.basicAck(deliveryTag, false);
                        }


                    });
        } catch (IOException e) {
            String msg = "Error consuming the message";
            throw new RuntimeException(msg, e);
        } catch (Exception e) {
            String msg = "Error connecting to broker";
            throw new RuntimeException(msg, e);
        }
    }

    public static void odomCallback(Agent agent, final Odometry_ msg) {
        // In the original program, odometry is published in robot frame, but it need velocities in
        // robot frame, and pose in global frame. So it directly cast the twist to base odom but
        // transformed the pose into global frame.
        // In our context both velocity and position are in odometry frame so only need to transform
        // velocity to robot frame. Currently map frame and odometry frame are the same.

        agent.me_lock_.lock();
        try {
            Twist_ twist = new Twist_();
            // here actually can just get the length of linear velocity instead
//            if (!tf_.transformTwist(base_frame_, global_frame_, msg.getTwist().getTwist(), twist)) {
//                node.getLog().error("Can not transform twist to base frame: " + msg.getHeader().getFrameId() + "->" + base_frame_);
//                return;
//            }
            twist.getLinear().setX(Vector2.abs(new Vector2(msg.getTwist().getLinear().getX(), msg.getTwist().getLinear().getY())));
            twist.getAngular().setZ(msg.getTwist().getAngular().getZ());

            agent.getBaseOdom().setTwist(twist);// base frame
            agent.getBaseOdom().setPose(msg.getPose());// global frame
            agent.getBaseOdom().setHeader(msg.getHeader());

            agent.setLast_seen_(msg.getHeader().getStamp());

            if ((System.currentTimeMillis() - agent.getLastTimeMePublished()) > agent.getPublishMePeriod()) {
                agent.setLastTimeMePublished(System.currentTimeMillis());
                try {
                    Methods_RMQ.publishMsg(
                            agent.getRmqMsgManager().getRMQContexts().get(Constant.KEY_POSE_SHARE),
                            getPoseShareMsg(agent).toJSON());
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        } finally {
            agent.me_lock_.unlock();
        }
    }

    public static PoseShareMsg_ getPoseShareMsg(Agent agent) {
        PoseShareMsg_ poseShareMsg_ = new PoseShareMsg_();
        poseShareMsg_.getHeader().setFrameId(agent.getBase_frame_());
        poseShareMsg_.getHeader().setStamp(System.currentTimeMillis());
        poseShareMsg_.setPose(agent.getBaseOdom().getPose());
        poseShareMsg_.setTwist(agent.getBaseOdom().getTwist());
        poseShareMsg_.setControlled(agent.getController());
        poseShareMsg_.setHoloRobot(agent.getHoloRobot());
        poseShareMsg_.setRadius(agent.getRadius() + agent.getCur_loc_unc_radius_());
        poseShareMsg_.setRobotId(agent.getRobotId());
        poseShareMsg_.setHolonomicVelocity(new Vector3d_(
                agent.getHolo_velocity_().getX(),
                agent.getHolo_velocity_().getY(),
                0));
        List<Vector3d_> footprint = new ArrayList<Vector3d_>();
        for (Vector2 vector2 : agent.getFootprint_minkowski()) {
            Vector3d_ vector3d_ = new Vector3d_(vector2.getX(), vector2.getY(), 0);
            footprint.add(vector3d_);
        }
        poseShareMsg_.setFootPrint_Minkowski(footprint);
        return poseShareMsg_;
    }


    public static void poseShareCallback(final Agent agent, final PoseShareMsg_ msg) {
        agent.getNeighbors_lock_().lock();
        try {
            String cur_id = msg.getRobotId();
            if (!cur_id.equals(agent.getRobotId())) {  //if it is not me do something
                int i;
                List<Agent> neighbors = agent.getAgentNeighbors();
                for (i = 0; i < neighbors.size(); i++) {
                    if (neighbors.get(i).getRobotId().equals(cur_id)) {
                        //I found the robot
                        break;
                    }
                }
                if (i >= neighbors.size()) { //Robot is new, so it will be added to the list
                    Agent new_robot = new Agent(cur_id);
                    new_robot.setHolo_robot_(msg.getHoloRobot());
                    neighbors.add(new_robot);
                    logger.info(agent.getRobotId() + " added a new neighbor with AgentName " + cur_id + " and radius " + msg.getRadius());
                }

                Agent lstagent = neighbors.get(i);
                lstagent.getBaseOdom().setPose(msg.getPose());
                lstagent.getBaseOdom().setTwist(msg.getTwist());

                lstagent.setHolo_velocity_(new Vector2(
                                msg.getHolonomicVelocity().getX(),
                                msg.getHolonomicVelocity().getY())
                );
                lstagent.setRadius(msg.getRadius());
                lstagent.setControlled(msg.getControlled());
                List<Vector2> footprint = new ArrayList<Vector2>();
                for (Vector3d_ vector3d_ : msg.getFootPrint_Minkowski())
                    footprint.add(new Vector2(vector3d_.getX(), vector3d_.getY()));
                lstagent.setFootprint_minkowski(footprint);
                lstagent.setLast_seen_(msg.getHeader().getStamp());
            }

        } finally {
            agent.getNeighbors_lock_().unlock();
        }
    }

    // scans are published in global frame
    public static void scanCallback(final Agent agent, final PointCloud2_ msg) {
        List<Vector3d_> point3ds = new ArrayList<Vector3d_>();

        if (msg.getWidth() * msg.getHeight() == 0) {
            agent.getObstacle_lock_().lock();
            try {
                agent.getObstacles_from_laser_().clear();
            } finally {
                agent.getObstacle_lock_().unlock();
            }
            return;
        }

        ChannelBuffer data = msg.getData().copy();
        while (data.readableBytes() > 0) {
            double[] pt = new double[msg.getDimension()];
            for (int k = 0; k < msg.getDimension(); k++) {
                pt[k] = data.readFloat();
            }
            point3ds.add(new Vector3d_(pt[0], pt[1], pt[2]));
        }
        //no need to transform rviz will transform automatically
//            if (!tf_.transformPoint3ds(global_frame_, base_frame_, point3ds, msg.getHeader().getStamp().totalNsecs(), dur_m)) {
//                node.getLog().error("Can not transform cloud points: " + base_frame_ + "->" + global_frame_);
//                return;
//            }

        agent.getObstacle_lock_().lock();
        try {
            agent.getObstacles_from_laser_().clear();
            double threshold_convex = 0.03;
            double threshold_concave = -0.03;
            //    ROS_ERROR("%d", (int)cloud.points.size());
            Vector2 start;
            for (int i = 0; i < point3ds.size(); i++) {
                start = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
                while (pointInNeighbor(agent, start) && i < point3ds.size() - 1) {
                    i++;
                    start = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
                }
                if (i == point3ds.size()) {
                    //it is a agent
                    return;
                }

                boolean found = false;
                Vector2 prev = new Vector2(start);
                double first_ang = 0;
                double prev_ang = 0;
                Vector2 next;
                while (!found) {
                    i++;
                    if (i == point3ds.size()) {
                        break;
                    }
                    next = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
                    while (pointInNeighbor(agent, next) && i < point3ds.size() - 1) {
                        i++;
                        next = new Vector2(point3ds.get(i).getX(), point3ds.get(i).getY());
                    }

                    if (Vector2.abs(Vector2.minus(next, prev)) > 2 * agent.getFootprint_radius_()) {
                        found = true;
                        break;
                    }
                    Vector2 dif = Vector2.minus(next, start);
                    double ang = Math.atan2(dif.getY(), dif.getX());
                    if (!prev.equals(start)) {
                        if (ang - first_ang < threshold_concave) {
                            found = true;
                            i -= 2;
                            break;
                        }
                        if (ang - prev_ang < threshold_concave) {
                            found = true;
                            i -= 2;
                            break;
                        }
                        if (ang - prev_ang > threshold_convex) { //going towards me
                            found = true;
                            i -= 2;
                            break;
                        }
                        if (ang - first_ang > threshold_convex) { //going towards me
                            found = true;
                            i -= 2;
                            break;
                        }

                    } else {
                        first_ang = ang;
                    }
                    prev = new Vector2(next);
                    prev_ang = ang;
                }
                Obstacle obst = new Obstacle(start, prev);
                agent.getObstacles_from_laser_().add(obst);
            }
        } finally {
            agent.getObstacle_lock_().unlock();
        }
    }


    private static boolean pointInNeighbor(final Agent agent, Vector2 point) {
        double dist;
        for (int i = 0; i < agent.getAgentNeighbors().size(); i++) {
            dist = Vector2.abs(Vector2.minus(point, agent.getAgentNeighbors().get(i).getPosition().getPos()));
            if (dist <= agent.getAgentNeighbors().get(i).getRadius()) {
                return true;
            }
        }
        return false;
    }

    // for test
    private static void particleCloudTestCallback(final Agent agent, final PoseArray_ msg) {
        // in robot base frame do not need transform
        double x, y;
        List<Vector2> localization_footprint = new ArrayList<Vector2>();
        List<Vector2> own_footprint = new ArrayList<Vector2>();
        for (int i = 0; i < msg.getPoses().size(); i++) {
            x = msg.getPoses().get(i).getPosition().getX();
            y = msg.getPoses().get(i).getPosition().getY();
            Vector2 p = new Vector2(x, y);
            if (p.getLength() > 0.1)
                continue;
            localization_footprint.add(p);
        }

        //for test replaced the algorithm computeNewMinkowskiFootprint
        for (int i = 0; i < agent.getFootprint_original().size(); i++) {
            Vector2 p = agent.getFootprint_original().get(i);
            own_footprint.add(new Vector2(p.getX(), p.getY()));
            //      ROS_WARN("footprint point p = (%f, %f) ", footprint_[i].x, footprint_[i].y);
        }
        agent.setFootprint_minkowski(Methods_Planners.minkowskiSumConvexHull(localization_footprint, own_footprint));
    }

}
