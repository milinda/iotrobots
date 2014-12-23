package cgl.iotrobots.collavoid.planners;

import cgl.iotrobots.collavoid.commons.rmqmsg.PoseStamped_;

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

    public boolean makePlan(final PoseStamped_ start, final PoseStamped_ goal, List<PoseStamped_> plan) {
        //start and goal are supposed not to change

        plan.clear();
        plan.add(start);
        double x, y, dir_x, dir_y;
        dir_x = goal.getPose().getPosition().getX() - start.getPose().getPosition().getX();
        dir_y = goal.getPose().getPosition().getY() - start.getPose().getPosition().getY();
        double length = Math.sqrt(dir_y * dir_y + dir_x * dir_x);
        dir_x /= length;
        dir_y /= length;
        x = start.getPose().getPosition().getX() + 0.1 * dir_x;
        y = start.getPose().getPosition().getY() + 0.1 * dir_y;

        while (Math.abs(x - goal.getPose().getPosition().getX()) > 0.2 || Math.abs(y - goal.getPose().getPosition().getY()) > 0.2) {
            PoseStamped_ pose = new PoseStamped_();
            pose.getPose().getPosition().setX(x);
            pose.getPose().getPosition().setY(y);
            pose.getHeader().setFrameId(goal.getHeader().getFrameId());
            pose.getHeader().setStamp(goal.getHeader().getStamp());
            plan.add(pose);
            x += 0.1 * dir_x;
            y += 0.1 * dir_y;
        }
        plan.add(goal);

        return true;
    }
}
