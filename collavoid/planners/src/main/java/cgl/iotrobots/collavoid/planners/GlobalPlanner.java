package cgl.iotrobots.collavoid.planners;

import cgl.iotrobots.collavoid.commons.rmqmsg.Header_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Pose_;

// should decouple ros related types
//import costmap_2d.VoxelGrid;
//import geometry_msgs.PoseStamped;
//import org.ros.internal.message.DefaultMessageFactory;
//import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
//import org.ros.message.MessageDefinitionProvider;
//import org.ros.message.MessageFactory;

import java.util.List;

public class GlobalPlanner {

//    void initialize(String name, VoxelGrid costmap_ros){
//    }

    public boolean makePlan(final Pose_ start, final Pose_ goal, List<Pose_> plan) {
        //start and goal are supposed not to change

        plan.clear();
        plan.add(start);
        double x, y, dir_x, dir_y;
        dir_x = goal.getPosition().getX() - start.getPosition().getX();
        dir_y = goal.getPosition().getY() - start.getPosition().getY();
        double length = Math.sqrt(dir_y * dir_y + dir_x * dir_x);
        dir_x /= length;
        dir_y /= length;
        x = start.getPosition().getX() + 0.1 * dir_x;
        y = start.getPosition().getY() + 0.1 * dir_y;

        while (Math.abs(x - goal.getPosition().getX()) > 0.2 || Math.abs(y - goal.getPosition().getY()) > 0.2) {
            Pose_ pose = new Pose_();
            pose.getPosition().setX(x);
            pose.getPosition().setY(y);
            pose.setHeader(new Header_(goal.getHeader().getFrameId(), goal.getHeader().getStamp()));
            plan.add(pose);
            x += 0.1 * dir_x;
            y += 0.1 * dir_y;
        }
        plan.add(goal);

        return true;
    }
}
