import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.Obstacle;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.PoseStamped_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Serializers;
import cgl.iotrobots.collavoid.commons.rmqmsg.StartGoal_;
import com.esotericsoftware.kryo.Kryo;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by hjh on 12/18/14.
 */
public class testSerialize {
    private static final Kryo kryo = new Kryo();
    public static void main(String[] args) throws IOException {
        StartGoal_ startGoal_ = new StartGoal_();
        startGoal_.setGoal(new PoseStamped_());
        startGoal_.setStart(new PoseStamped_());

        byte[] body = startGoal_.toJSON();

        StartGoal_ res = Serializers.JSONToStartGoal_(body);

        System.out.println(res.getStart().toString());

        kryo.register(Agent.class);

        Agent agent = new Agent("test");
//        agent.Name="test";

        byte[] agentbyte = Serializers.serialize(kryo, agent);

        System.out.println("----------------agent serialized to a byte array of length:" + agentbyte.length);

        Object deSerializeObject = Serializers.deSerialize(kryo, agentbyte, Agent.class);


        if (deSerializeObject instanceof Agent) {
            Agent newagent = (Agent) deSerializeObject;
            System.out.println("Deserialize " + newagent.Name);
        }
        

    }
}
