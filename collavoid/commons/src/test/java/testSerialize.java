import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.VO;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.rmqmsg.PoseStamped_;
import cgl.iotrobots.collavoid.commons.rmqmsg.BaseConfig_;
import com.esotericsoftware.kryo.Kryo;

import java.io.IOException;

/**
 * Created by hjh on 12/18/14.
 */
public class testSerialize {
    private static final Kryo kryo = new Kryo();
    public static void main(String[] args) throws IOException {
        BaseConfig_ baseConfig_ = new BaseConfig_();
        baseConfig_.setGoal(new PoseStamped_());
        baseConfig_.setStart(new PoseStamped_());

        byte[] body = Methods_RMQ.serialize(baseConfig_);

        BaseConfig_ res = Serializers.JSONToStartGoal_(body);

        System.out.println(res.getStart().toString());

        kryo.register(Agent.class);

        Agent agent = new Agent("test");
        agent.voAgents.add(new VO());
//        agent.Name="test";

        byte[] agentbyte = Serializers.serialize(kryo, agent);

        System.out.println("----------------agent serialized to a byte array of length:" + agentbyte.length);

        Object deSerializeObject = Serializers.deSerialize(kryo, agentbyte, Agent.class);


        if (deSerializeObject instanceof Agent) {
            Agent newagent = (Agent) deSerializeObject;
            System.out.println("Deserialize " + newagent.Name);
        }

        testFinal(new Vector2());

    }

    public static void testFinal(final Vector2 vector2) {
        vector2.setX(vector2.getX() + 2);

    }
    
    
}
