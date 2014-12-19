package cgl.iotrobots.collavoid.controller;

import cgl.iotrobots.collavoid.commons.*;
import com.rabbitmq.client.AMQP;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.Twist;
import nav_msgs.Odometry;
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

    private String exchangeName;

    private String odomRoutingKey;

    private String scanRoutingKey;

    private String particleRoutingKey;

    private String velRoutingKey;

    private String velConQueueName;

    private Channel channel;

    private boolean autoAck = false;

    private BlockingQueue<Twist_> velQueue = new LinkedBlockingDeque<Twist_>();

    public AgentROSNode(String name, Channel channel, Map<String, String> RMQParams) {
        this.nodeName = name;
        this.channel = channel;
        this.exchangeName = RMQParams.get(Constants.KEY_EXCHAGE_NAME);
        this.odomRoutingKey = RMQParams.get(Constants.KEY_ROUTINGKEY_ODOMETRY);
        this.scanRoutingKey = RMQParams.get(Constants.KEY_ROUTINGKEY_SCAN);
        this.particleRoutingKey = RMQParams.get(Constants.KEY_ROUTINGKEY_PARTICLE);
        this.velRoutingKey = RMQParams.get(Constants.KEY_ROUTINGKEY_VELOCITY);

        BindQueue(RMQParams);
    }

    private void BindQueue(Map<String, String> RMQParams) {
        try {
            for (Map.Entry e : RMQParams.entrySet()) {
                if (e.getKey().equals(Constants.KEY_EXCHAGE_NAME))
                    continue;
                else if (e.getKey().equals(Constants.KEY_ROUTINGKEY_VELOCITY)) {
                    velConQueueName = channel.queueDeclare().getQueue();
                    channel.queueBind(velConQueueName, exchangeName, velRoutingKey);
                } else
                    channel.queueBind(channel.queueDeclare().getQueue(), exchangeName, (String) e.getValue());
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        String robotNodeName = new String(nodeName).replace("rmq", "");
        final Publisher<Twist> velcmdPublisher =
                connectedNode.newPublisher(robotNodeName + "/cmd_vel", Twist._TYPE);
        final Subscriber<Odometry> odometrySubscriber =
                connectedNode.newSubscriber(robotNodeName + "/odometry", Odometry._TYPE);
        final Subscriber<PointCloud2> laserScanSubscriber =
                connectedNode.newSubscriber(robotNodeName + "/scan/point_cloud2", PointCloud2._TYPE);
        final Subscriber<PoseArray> poseArraySubscriber =
                connectedNode.newSubscriber(robotNodeName + "/particlecloud", PoseArray._TYPE);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException ie) {
            ie.printStackTrace();
        }

        try {
            channel.basicConsume(velConQueueName, autoAck, velRoutingKey + "Tag",
                    new DefaultConsumer(channel) {
                        @Override
                        public void handleDelivery(String consumerTag,
                                                   Envelope envelope,
                                                   AMQP.BasicProperties properties,
                                                   byte[] body)
                                throws IOException {
                            long deliveryTag = envelope.getDeliveryTag();
                            Twist_ velocity = CommonUtils.JSONToTwist_(body);
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
                Odometry_ odometry_ = new Odometry_(odometry);
                try {
                    channel.basicPublish(exchangeName, odomRoutingKey, null, odometry_.toJSON());
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        laserScanSubscriber.addMessageListener(new MessageListener<PointCloud2>() {
            @Override
            public void onNewMessage(PointCloud2 pointCloud2) {
                PointCloud2_ pts = new PointCloud2_();
                pts.setWith(pointCloud2.getWidth());
                pts.setHeight(pointCloud2.getHeight());
                pts.setDimension(pointCloud2.getFields().size());
                pts.setData(pointCloud2.getData());
                try {
                    channel.basicPublish(exchangeName, scanRoutingKey, null, pts.toJSON());
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        poseArraySubscriber.addMessageListener(new MessageListener<PoseArray>() {
            @Override
            public void onNewMessage(PoseArray poseArray) {
                PoseArray_ pa = new PoseArray_();
                pa.setHeader(poseArray.getHeader().getFrameId(),
                        poseArray.getHeader().getStamp().nsecs);

                List<Pose_> poses = new ArrayList<Pose_>();
                for (Pose pose : poseArray.getPoses()) {
                    poses.add(new Pose_(pose));
                }
                pa.setPoses(poses);

                try {
                    channel.basicPublish(exchangeName, particleRoutingKey, null, pa.toJSON());
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    @Override
    public void onShutdown(Node node) {
        node.shutdown();
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(nodeName);
    }
}
