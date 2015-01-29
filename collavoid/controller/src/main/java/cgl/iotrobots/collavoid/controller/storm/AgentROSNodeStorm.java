package cgl.iotrobots.collavoid.controller.storm;

import cgl.iotrobots.collavoid.commons.TimeDelayAnalysis.Constants;
import cgl.iotrobots.collavoid.commons.TimeDelayAnalysis.TimeDelayRecorder;
import cgl.iotrobots.collavoid.commons.planners.Parameters;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import com.esotericsoftware.kryo.Kryo;
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
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingDeque;

public class AgentROSNodeStorm extends AbstractNodeMain {

    private String sensorID = "robot";

    private String nodeName = "";

    private boolean autoAck = false;

    private boolean goalReached;

    private BlockingQueue<Twist_> velQueue = new LinkedBlockingDeque<Twist_>();

    private Map<String, RMQContext> rmqContexts;

    private BaseConfig_ baseConfig = new BaseConfig_();

    private Publisher<Twist> velcmdPublisher;

    private Subscriber<Odometry> odometrySubscriber;

    private Subscriber<PointCloud2> laserScanSubscriber;

    private Subscriber<PoseArray> poseArraySubscriber;

    private Subscriber<PoseStamped> startGoalSubscriber;

    final Map<String, TimeDelayRecorder> delayRecorders = new HashMap<>();

    public AgentROSNodeStorm(String nodeName, Map<String, RMQContext> msgContexts) {
        this.nodeName = nodeName;
        // running controller node need a different node name however topics and other stuffs are
        // using original node name which is also used as the sensorid. So need to get rid of the suffix.
        this.sensorID = new String(nodeName).replace("_rmq", "");
        rmqContexts = msgContexts;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        velcmdPublisher =
                connectedNode.newPublisher(sensorID + "/cmd_vel", Twist._TYPE);
        odometrySubscriber =
                connectedNode.newSubscriber(sensorID + "/odometry", Odometry._TYPE);
        laserScanSubscriber =
                connectedNode.newSubscriber(sensorID + "/scan/point_cloud2", PointCloud2._TYPE);
        poseArraySubscriber =
                connectedNode.newSubscriber(sensorID + "/particlecloud", PoseArray._TYPE);
        startGoalSubscriber =
                connectedNode.newSubscriber(sensorID + "/start_goal", PoseStamped._TYPE);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException ie) {
            ie.printStackTrace();
        }

        getTimeDelayRecorders();

        try {
            String queueName = rmqContexts.get(Constant_msg.KEY_VELOCITY_CMD).QUEUE_NAME;
            String routingKey = rmqContexts.get(Constant_msg.KEY_VELOCITY_CMD).ROUTING_KEY;
            final Channel velCmdChannel = rmqContexts.get(Constant_msg.KEY_VELOCITY_CMD).CHANNEL;
            velCmdChannel.basicConsume(queueName, autoAck, routingKey + "Tag",
                    new DefaultConsumer(velCmdChannel) {
                        TimeDelayRecorder delayRecorder_ = delayRecorders.get(Constant_msg.KEY_VELOCITY_CMD);
                        Kryo kryo = Methods_RMQ.getKryo();
                        @Override
                        public void handleDelivery(String consumerTag,
                                                   Envelope envelope,
                                                   AMQP.BasicProperties properties,
                                                   byte[] body)
                                throws IOException {
                            long deliveryTag = envelope.getDeliveryTag();
                            Twist_ velocity = (Twist_) Methods_RMQ.deSerialize(kryo, body);
                            delayRecorder_.append(
                                    sensorID,
                                    velocity.getTime(),
                                    System.currentTimeMillis());
                            goalReached = velocity.isGoalReached();
                            velQueue.offer(velocity);
                            velCmdChannel.basicAck(deliveryTag, false);
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
            int j = 0;
            double sum;
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

//                double delay = (System.currentTimeMillis() - m.getTime()) / 1000.0;
//                System.out.println("Delay for "+nodeName+": "+delay);
//                if (delay > 1) {
//                    Methods_RMQ.clearQueues(rmqContexts);
//                    velQueue.clear();
//                    System.out.println("Delay is longer than 1s, " + "queue purged!!");
//                }

            }
        });

        odometrySubscriber.addMessageListener(new MessageListener<Odometry>() {
            private Kryo kryo = Methods_RMQ.getKryo();
            TimeDelayRecorder delayRecorder_ = delayRecorders.get(Constant_msg.KEY_ODOMETRY);
            @Override
            public void onNewMessage(Odometry odometry) {
                delayRecorder_.append(
                        sensorID,
                        odometry.getHeader().getStamp().totalNsecs() / 1000000,
                        System.currentTimeMillis());
                Methods_RMQ.publishMsg(
                        rmqContexts.get(Constant_msg.KEY_ODOMETRY),
//                        Methods_RMQ.serialize(toOdometry_(odometry)));
                        Methods_RMQ.serialize(kryo, toOdometry_(odometry)));
            }
        });

        laserScanSubscriber.addMessageListener(new MessageListener<PointCloud2>() {
            private Kryo kryo = Methods_RMQ.getKryo();
            TimeDelayRecorder delayRecorder_ = delayRecorders.get(Constant_msg.KEY_SCAN);
            @Override
            public void onNewMessage(PointCloud2 pointCloud2) {
                delayRecorder_.append(
                        sensorID,
                        pointCloud2.getHeader().getStamp().totalNsecs() / 1000000,
                        System.currentTimeMillis());

                Methods_RMQ.publishMsg(
                        rmqContexts.get(Constant_msg.KEY_SCAN),
//                        Methods_RMQ.serialize(toPointCloud2_(pointCloud2)));
                        Methods_RMQ.serialize(kryo, toPointCloud2_(pointCloud2)));
            }
        });

        poseArraySubscriber.addMessageListener(new MessageListener<PoseArray>() {
            TimeDelayRecorder delayRecorder_ = delayRecorders.get(Constant_msg.KEY_POSE_ARRAY);
            private Kryo kryo = Methods_RMQ.getKryo();
            @Override
            public void onNewMessage(PoseArray poseArray) {
                delayRecorder_.append(
                        sensorID,
                        poseArray.getHeader().getStamp().totalNsecs() / 1000000,
                        System.currentTimeMillis());

                Methods_RMQ.publishMsg(
                        rmqContexts.get(Constant_msg.KEY_POSE_ARRAY),
//                        Methods_RMQ.serialize(toPoseArray_(poseArray)));
                        Methods_RMQ.serialize(kryo, toPoseArray_(poseArray)));
            }
        });

        startGoalSubscriber.addMessageListener(new MessageListener<PoseStamped>() {
            TimeDelayRecorder delayRecorder_ = delayRecorders.get(Constant_msg.KEY_BASE_CONFIG);
            @Override
            public void onNewMessage(PoseStamped msg) {
                PoseStamped_ poseStamped_ = toPoseStamped_(msg);
                if (msg.getHeader().getSeq() % 2 == 0) {
                    baseConfig.setStart(poseStamped_);
                } else {
                    baseConfig.setGoal(poseStamped_);
                }

                if (msg.getHeader().getSeq() >= 9) {
                    delayRecorder_.append(
                            sensorID,
                            msg.getHeader().getStamp().totalNsecs() / 1000000,
                            System.currentTimeMillis());

                    baseConfig.setControlFreq(Parameters.CONTROLLER_FREQUENCY);
                    baseConfig.setPublisMeFreq(Parameters.PUBLISH_ME_FREQUENCY);
                    baseConfig.setId(sensorID);
                    baseConfig.setTime(poseStamped_.getHeader().getStamp());
                    Methods_RMQ.publishMsg(
                            rmqContexts.get(Constant_msg.KEY_BASE_CONFIG),
                            Methods_RMQ.serialize(baseConfig));
                }
            }
        });
    }

    // delay test
    private void getTimeDelayRecorders() {
        delayRecorders.put(Constant_msg.KEY_ODOMETRY, new TimeDelayRecorder(
                Constants.PARAMETER_DELAY,
                Constant_msg.KEY_ODOMETRY,
                Constant_storm.Components.CONTROLLER_COMPONENT
        ));
        delayRecorders.put(Constant_msg.KEY_BASE_CONFIG, new TimeDelayRecorder(
                Constants.PARAMETER_DELAY,
                Constant_msg.KEY_BASE_CONFIG,
                Constant_storm.Components.CONTROLLER_COMPONENT
        ));
        delayRecorders.put(Constant_msg.KEY_SCAN, new TimeDelayRecorder(
                Constants.PARAMETER_DELAY,
                Constant_msg.KEY_SCAN,
                Constant_storm.Components.CONTROLLER_COMPONENT
        ));
        delayRecorders.put(Constant_msg.KEY_POSE_ARRAY, new TimeDelayRecorder(
                Constants.PARAMETER_DELAY,
                Constant_msg.KEY_POSE_ARRAY,
                Constant_storm.Components.CONTROLLER_COMPONENT
        ));
        delayRecorders.put(Constant_msg.KEY_VELOCITY_CMD, new TimeDelayRecorder(
                Constants.COMPUTATION_DELAY,
                Constant_msg.KEY_VELOCITY_CMD,
                Constant_storm.Components.CONTROLLER_COMPONENT
        ));
        for (Map.Entry<String, TimeDelayRecorder> e : delayRecorders.entrySet())
            e.getValue().open(false);
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
        odometry_.setId(sensorID);
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
        pts.setId(sensorID);
        return pts;
    }

    private PoseArray_ toPoseArray_(PoseArray poseArray) {
        PoseArray_ pa = new PoseArray_();
        pa.getHeader().setFrameId(poseArray.getHeader().getFrameId());
        pa.getHeader().setStamp(poseArray.getHeader().getStamp().totalNsecs() / 1000000);

        List<Pose_> poses = new ArrayList<Pose_>();
        for (Pose pose : poseArray.getPoses()) {
            poses.add(toPose_(pose));
        }
        pa.setPoses(poses);
        pa.setId(sensorID);
        return pa;
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

    public boolean isGoalReached() {
        return goalReached;
    }

    public BlockingQueue<Twist_> getVelQueue() {
        return velQueue;
    }

    @Override
    public void onShutdown(Node node) {
        for (Map.Entry<String, TimeDelayRecorder> e : delayRecorders.entrySet())
            e.getValue().close();
        velcmdPublisher.shutdown();
        odometrySubscriber.shutdown();
        poseArraySubscriber.shutdown();
        startGoalSubscriber.shutdown();
        laserScanSubscriber.shutdown();
        node.shutdown();
    }

    @Override
    public GraphName getDefaultNodeName() {
        // should be the same as node name or will not work right
        return GraphName.of(nodeName);
    }
}
