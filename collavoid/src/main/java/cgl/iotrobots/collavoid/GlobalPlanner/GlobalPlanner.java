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

/**
 * Created by hjh on 11/5/14.
 */
public class GlobalPlanner {

    private static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    private static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);
    private static Pose pose=messageFactory.newFromType(Pose._TYPE);
    private static Point pos=messageFactory.newFromType(Point._TYPE);
    private static Quaternion ori=messageFactory.newFromType(Quaternion._TYPE);//for temporary assign use

    void initialize(String name, VoxelGrid costmap_ros){
    }

    boolean makePlan(ConnectedNode node,final PoseStamped start,final PoseStamped goal, List<PoseStamped> plan){
        //start and goal are supposed not to change
        node.getLog().debug(String.format("Got a start: %1$.2f, %2$.2f, and a goal: %3$.2f, %4$.2f",
                start.getPose().getPosition().getX(),
                start.getPose().getPosition().getY(),
                goal.getPose().getPosition().getX(),
                goal.getPose().getPosition().getY()));

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
        node.getLog().debug(String.format("dir: %1$.2f, %2$.2f, cur: %3$.2f, %4$.2f", dir_x, dir_y, x, y));

        while (Math.abs(x - goal.getPose().getPosition().getX()) > 0.2 || Math.abs(y - goal.getPose().getPosition().getY()) > 0.2) {
            PoseStamped point=messageFactory.newFromType(PoseStamped._TYPE);
            pos.setX(x);
            pos.setY(y);
            ori.setW(1);// complex form while theta=0 real part w=cos(theta/2)=1
            pose.setPosition(pos);
            pose.setOrientation(ori);

            point.setPose(pose);
            plan.add(point);
            x += 0.1 * dir_x;
            y += 0.1 * dir_y;
        }
        plan.add(goal);

        return true;
    }
}
