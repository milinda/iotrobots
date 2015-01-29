package cgl.iotrobots.collavoid.controller.original;

import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import com.rabbitmq.client.AMQP;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.PoseStamped;
import geometry_msgs.Twist;
import nav_msgs.Odometry;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import sensor_msgs.PointCloud2;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingDeque;

public class AgentROSNode extends AbstractNodeMain {

    private String nodeName = "robot";

    private Channel channel;

    private boolean autoAck = false;

    private BlockingQueue<Twist_> velQueue = new LinkedBlockingDeque<Twist_>();

    private Map<String, RMQContext> Contexts;

    private BaseConfig_ startGoal = new BaseConfig_();

    private Publisher<Twist> velcmdPublisher;

    private Subscriber<Odometry> odometrySubscriber;

    private Subscriber<PointCloud2> laserScanSubscriber;

    private Subscriber<PoseArray> poseArraySubscriber;

    private Subscriber<PoseStamped> startGoalSubscriber;

    public AgentROSNode(String name, Channel channel, Map<String, RMQContext> msgContexts) {
        this.nodeName = name;
        this.channel = channel;
        Contexts = msgContexts;
        BindQueue(Contexts);
    }

    private void BindQueue(Map<String, RMQContext> RMQParams) {
        try {
            for (Map.Entry<String, RMQContext> e : RMQParams.entrySet()) {
                e.getValue().CHANNEL = channel;
                e.getValue().CHANNEL.exchangeDeclare(
                        e.getValue().EXCHANGE_NAME,
                        e.getValue().EXCHANGE_TYPE,
                        e.getValue().DURABLE
                );
                channel.queueDeclare(e.getValue().QUEUE_NAME, false, false, false, null);
                channel.queueBind(e.getValue().QUEUE_NAME, e.getValue().EXCHANGE_NAME, e.getValue().ROUTING_KEY);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        final String robotNodeName = new String(nodeName).replace("_rmq", "");
        velcmdPublisher =
                connectedNode.newPublisher(robotNodeName + "/cmd_vel", Twist._TYPE);
        odometrySubscriber =
                connectedNode.newSubscriber(robotNodeName + "/odometry", Odometry._TYPE);
        laserScanSubscriber =
                connectedNode.newSubscriber(robotNodeName + "/scan/point_cloud2", PointCloud2._TYPE);
        poseArraySubscriber =
                connectedNode.newSubscriber(robotNodeName + "/particlecloud", PoseArray._TYPE);
        startGoalSubscriber =
                connectedNode.newSubscriber(robotNodeName + "/start_goal", PoseStamped._TYPE);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException ie) {
            ie.printStackTrace();
        }

        try {
            String queueName = Contexts.get(Constant_msg.KEY_VELOCITY_CMD).QUEUE_NAME;
            String routingKey = Contexts.get(Constant_msg.KEY_VELOCITY_CMD).ROUTING_KEY;
            channel.basicConsume(queueName, autoAck, routingKey + "Tag",
                    new DefaultConsumer(channel) {
                        @Override
                        public void handleDelivery(String consumerTag,
                                                   Envelope envelope,
                                                   AMQP.BasicProperties properties,
                                                   byte[] body)
                                throws IOException {
                            long deliveryTag = envelope.getDeliveryTag();
                            Twist_ velocity = (Twist_) Methods_RMQ.deSerialize(body, Twist_.class);
                            velQueue.offer(velocity);
                            channel.basicAck(deliveryTag, false);
                        }
                    });
        } catch (IOException e) {
            String msg = "Error consuming the message";
            throw new RuntimeException(msg, e);
        } catch (Exception e) {
            String msg = "Error connecting to broker";
            throw new RuntimeException(msg, e);
        }

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                Twist_ m = velQueue.take();
                Twist str = velcmdPublisher.newMessage();
                str.getLinear().setX(m.getLinear().getX());
                str.getLinear().setY(m.getLinear().getY());
                str.getLinear().setZ(m.getLinear().getZ());

                str.getAngular().setX(m.getAngular().getX());
                str.getAngular().setY(m.getAngular().getY());
                str.getAngular().setZ(m.getAngular().getZ());

                velcmdPublisher.publish(str);
            }
        });

        odometrySubscriber.addMessageListener(new MessageListener<Odometry>() {
            @Override
            public void onNewMessage(Odometry odometry) {
                if (channel.isOpen())
                try {
                    channel.basicPublish(
                            Contexts.get(Constant_msg.KEY_ODOMETRY).EXCHANGE_NAME,
                            Contexts.get(Constant_msg.KEY_ODOMETRY).ROUTING_KEY,
                            null,
                            Methods_RMQ.serialize(toOdometry_(odometry)));
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        laserScanSubscriber.addMessageListener(new MessageListener<PointCloud2>() {
            @Override
            public void onNewMessage(PointCloud2 pointCloud2) {
                if (channel.isOpen())
                try {
                    channel.basicPublish(
                            Contexts.get(Constant_msg.KEY_SCAN).EXCHANGE_NAME,
                            Contexts.get(Constant_msg.KEY_SCAN).ROUTING_KEY,
                            null,
                            Methods_RMQ.serialize(toPointCloud2_(pointCloud2)));
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        poseArraySubscriber.addMessageListener(new MessageListener<PoseArray>() {
            @Override
            public void onNewMessage(PoseArray poseArray) {
                PoseArray_ pa = new PoseArray_();
                pa.getHeader().setFrameId(poseArray.getHeader().getFrameId());
                pa.getHeader().setStamp(poseArray.getHeader().getStamp().totalNsecs() / 1000000);

                List<Pose_> poses = new ArrayList<Pose_>();
                for (Pose pose : poseArray.getPoses()) {
                    poses.add(toPose_(pose));
                }
                pa.setPoses(poses);
                if (channel.isOpen())
                try {
                    channel.basicPublish(
                            Contexts.get(Constant_msg.KEY_POSE_ARRAY).EXCHANGE_NAME,
                            Contexts.get(Constant_msg.KEY_POSE_ARRAY).ROUTING_KEY,
                            null,
                            Methods_RMQ.serialize(pa));
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        startGoalSubscriber.addMessageListener(new MessageListener<PoseStamped>() {
            RMQContext startGoalContext = Contexts.get(Constant_msg.KEY_START_GOAL);
            @Override
            public void onNewMessage(PoseStamped msg) {
                PoseStamped_ poseStamped_ = toPoseStamped_(msg);
                if (msg.getHeader().getSeq() % 2 == 0) {
                    startGoal.setStart(poseStamped_);
                } else {
                    startGoal.setGoal(poseStamped_);
                }

                if (msg.getHeader().getSeq() >= 9) {
                    Methods_RMQ.publishMsg(startGoalContext, Methods_RMQ.serialize(startGoal));
                }
            }
        });
    }

    private Odometry_ toOdometry_(Odometry odometry) {
        Pose_ pose_ = toPose_(odometry.getPose().getPose());

        Vector3d_ angular = new Vector3d_(
                odometry.getTwist().getTwist().getAngular().getX(),
                odometry.getTwist().getTwist().getAngular().getY(),
                odometry.getTwist().getTwist().getAngular().getZ()
        );
        Vector3d_ linear = new Vector3d_(
                odometry.getTwist().getTwist().getLinear().getX(),
                odometry.getTwist().getTwist().getLinear().getY(),
                odometry.getTwist().getTwist().getLinear().getZ()
        );
        Twist_ twist_ = new Twist_();
        twist_.setLinear(linear);
        twist_.setAngular(angular);

        Odometry_ odometry_ = new Odometry_();
        odometry_.setPose(pose_);
        odometry_.setTwist(twist_);
        odometry_.setChildFrameId(odometry.getChildFrameId());
        odometry_.getHeader().setFrameId(odometry.getHeader().getFrameId());
        odometry_.getHeader().setStamp(odometry.getHeader().getStamp().totalNsecs() / 1000000);

        return odometry_;
    }

    private PointCloud2_ toPointCloud2_(PointCloud2 pointCloud2) {
        PointCloud2_ pts = new PointCloud2_();
        pts.setWidth(pointCloud2.getWidth());
        pts.setHeight(pointCloud2.getHeight());
        pts.setDimension(pointCloud2.getFields().size());
        int pointNumber = pts.getWidth() * pts.getHeight();
        double[] data;
        if (pointNumber == 0)
            data = new double[0];
        else {
            data = new double[pointNumber * 3];
            int i = 0;
            ChannelBuffer databuffer = pointCloud2.getData().copy();
            while (databuffer.readableBytes() > 0)
                data[i++] = (double) databuffer.readFloat();
        }
        pts.setData(data);
        pts.getHeader().setStamp(pointCloud2.getHeader().getStamp().totalNsecs() / 1000000);
        pts.getHeader().setFrameId(pointCloud2.getHeader().getFrameId());

        return pts;
    }

    private Pose_ toPose_(Pose pose) {
        Pose_ pose_ = new Pose_();
        Vector3d_ position = new Vector3d_(
                pose.getPosition().getX(),
                pose.getPosition().getY(),
                pose.getPosition().getZ()
        );
        Vector4d_ orientation = new Vector4d_(
                pose.getOrientation().getX(),
                pose.getOrientation().getY(),
                pose.getOrientation().getZ(),
                pose.getOrientation().getW()
        );
        pose_.setOrientation(orientation);
        pose_.setPosition(position);
        return pose_;
    }

    private PoseStamped_ toPoseStamped_(PoseStamped msg) {
        PoseStamped_ poseStamped_ = new PoseStamped_();
        poseStamped_.getHeader().setFrameId(msg.getHeader().getFrameId());
        poseStamped_.getHeader().setStamp(msg.getHeader().getStamp().totalNsecs() / 1000000);
        Vector3d_ position = new Vector3d_(
                msg.getPose().getPosition().getX(),
                msg.getPose().getPosition().getY(),
                msg.getPose().getPosition().getZ()
        );
        Vector4d_ orientation = new Vector4d_(
                msg.getPose().getOrientation().getX(),
                msg.getPose().getOrientation().getY(),
                msg.getPose().getOrientation().getZ(),
                msg.getPose().getOrientation().getW()
        );
        poseStamped_.getPose().setPosition(position);
        poseStamped_.getPose().setOrientation(orientation);

        return poseStamped_;

    }

    @Override
    public void onShutdown(Node node) {
        velcmdPublisher.shutdown();
        odometrySubscriber.shutdown();
        poseArraySubscriber.shutdown();
        startGoalSubscriber.shutdown();
        laserScanSubscriber.shutdown();
        node.shutdown();
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(nodeName);
    }
}
