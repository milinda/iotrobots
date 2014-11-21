package test.node;

import collavoid_msgs.pose_twist_covariance_msgs;
import geometry_msgs.Point32;
import geometry_msgs.PolygonStamped;
import nav_msgs.Odometry;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by hjh on 11/21/14.
 */
public class PositionShareMux extends AbstractNodeMain {
    private static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    public static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);
    public static double footprint_radius_ = 0.17;
//    private String base_frame_;
    private ParameterTree params;
    private ConnectedNode node;
    private Publisher<pose_twist_covariance_msgs> pspub;


    @Override
    public void onStart(ConnectedNode connectedNode) {
        node=connectedNode;
        List<Subscriber<Odometry>> odomsubs = new ArrayList<Subscriber<Odometry>>();
        pspub= connectedNode.newPublisher("/position_share", pose_twist_covariance_msgs._TYPE);

        params = connectedNode.getParameterTree();

        for (int i = 1; i < 10; i++) {
            String robotId = "robot" + i;
            if (params.has(robotId)) {
                Subscriber<Odometry> odomsub = connectedNode.newSubscriber("/robot" + i + "/odom", Odometry._TYPE);
                odomsub.addMessageListener(new MessageListener<Odometry>() {
                    @Override
                    public void onNewMessage(Odometry msg) {
                        odomsubCallback(msg);
                    }

                });
                odomsubs.add(odomsub);
                System.out.println(robotId);
            }

        }
        for (int i = 0; i <odomsubs.size() ; i++) {
            System.out.println(odomsubs.get(i).getTopicName());
        }

    }

    private void odomsubCallback(Odometry msg) {
        String robotID=msg.getHeader().getFrameId().substring(0,6);

        String base_frame_=new String(robotID+"/base_link");

        pose_twist_covariance_msgs me_msg=pspub.newMessage();

        me_msg.getHeader().setStamp(node.getCurrentTime());
        me_msg.getHeader().setFrameId(base_frame_);

        me_msg.getPose().setPose(msg.getPose().getPose());
        me_msg.getTwist().setTwist(msg.getTwist().getTwist());

        me_msg.setControlled(true);
        me_msg.getHolonomicVelocity().setX(1.0);
        me_msg.getHolonomicVelocity().setY(1.0);

        me_msg.setHoloRobot(false);
        me_msg.setRadius((float)(footprint_radius_));
        me_msg.setRobotId(robotID);

        PolygonStamped fp=getFootprint();
        fp.getHeader().setFrameId(base_frame_);
        fp.getHeader().setStamp(node.getCurrentTime());
        me_msg.setFootprint(fp);

        pspub.publish(me_msg);

    }


    private static PolygonStamped getFootprint() {
        double angle = 0;
        double step = 2 * Math.PI / 72;
        List<Point32> points = new ArrayList<Point32>();
        while (angle < 2 * Math.PI) {
            Point32 pt = messageFactory.newFromType(Point32._TYPE);
            pt.setX((float) (0.17 * Math.cos(angle)));
            pt.setY((float) (footprint_radius_ * Math.sin(angle)));
            pt.setZ(0.0f);
            points.add(pt);
            angle += step;
        }

        PolygonStamped footprint = messageFactory.newFromType(PolygonStamped._TYPE);
        footprint.getPolygon().setPoints(points);
        return footprint;

    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("psmux");
    }
}
