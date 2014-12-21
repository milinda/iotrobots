package cgl.iotrobots.collavoid.planners;

import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;

public class MsgCallBacks {

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
                Methods.publishMsg(
                        agent.getRmqMsgManager().getShareChannel(),
                        agent.getRmqMsgManager().getRMQContexts().get(Constant.KEY_POSE_SHARE), );
            }
        } finally {
            agent.me_lock_.unlock();
        }
    }
}
