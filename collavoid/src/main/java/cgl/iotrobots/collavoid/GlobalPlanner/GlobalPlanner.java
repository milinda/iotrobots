package cgl.iotrobots.collavoid.GlobalPlanner;

/*BSD LICENSE*/

import costmap_2d.VoxelGrid;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.Quaternion;
import geometry_msgs.Point;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;

import java.util.List;

public class GlobalPlanner {

    private static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    private static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);


    void initialize(String name, VoxelGrid costmap_ros){
    }

    //public boolean makePlan(ConnectedNode node,final PoseStamped start,final PoseStamped goal, List<PoseStamped> plan){
    public boolean makePlan(final PoseStamped start,final PoseStamped goal, List<PoseStamped> plan){
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
            PoseStamped point=messageFactory.newFromType(PoseStamped._TYPE);
            point.setHeader(goal.getHeader());
            point.getPose().getPosition().setX(x);
            point.getPose().getPosition().setY(y);
            point.getPose().setOrientation(start.getPose().getOrientation());

            plan.add(point);
            x += 0.1 * dir_x;
            y += 0.1 * dir_y;
        }
        plan.add(goal);

        return true;
    }
}
