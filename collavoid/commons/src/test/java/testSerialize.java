import cgl.iotcloud.core.msg.MessageContext;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.VO;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import com.esotericsoftware.kryo.Kryo;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by hjh on 12/18/14.
 */
public class testSerialize {
    public static void main(String[] args) throws IOException {
        List<Object> list = new ArrayList<>();

        Kryo kryo = Methods_RMQ.getKryo();
        Kryo kryo1 = Methods_RMQ.getKryo();
        Twist_ twist_ = new Twist_();

        byte[] body = Methods_RMQ.serialize(kryo, twist_);
        MessageContext msgcontext = new MessageContext("11", body);
        System.out.println("serialized to bytes: " + body.length);

        System.out.println(Arrays.toString(body));

        byte[] body1 = msgcontext.getBody();
        Twist_ deser = (Twist_) Methods_RMQ.deSerialize(kryo1, body1, Twist_.class);
        System.out.println("dese: " + deser.toString());
    }

    public static void testFinal(final Vector2 vector2) {
        vector2.setX(vector2.getX() + 2);

    }
    
    
}
