import cgl.iotrobots.collavoid.commons.planners.Obstacle;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.PoseStamped_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Serializers;
import cgl.iotrobots.collavoid.commons.rmqmsg.StartGoal_;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by hjh on 12/18/14.
 */
public class testSerialize {
    public static void main(String[] args) throws IOException {
        StartGoal_ startGoal_ = new StartGoal_();
        startGoal_.setGoal(new PoseStamped_());
        startGoal_.setStart(new PoseStamped_());

        byte[] body = startGoal_.toJSON();

        StartGoal_ res = Serializers.JSONToStartGoal_(body);

        System.out.println(res.getStart().toString());
        

    }
}
