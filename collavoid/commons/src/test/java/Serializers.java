import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;
import com.esotericsoftware.kryo.serializers.MapSerializer;
import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.HashMap;

public class Serializers {

    public static Twist_ JSONToTwist_(byte[] data) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        return mapper.readValue(data, Twist_.class);
    }

    public static Odometry_ JSONToOdometry_(byte[] data) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        return mapper.readValue(data, Odometry_.class);
    }

    public static PoseShareMsg_ JSONToPoseShareMsg_(byte[] data) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        return mapper.readValue(data, PoseShareMsg_.class);
    }

    public static PointCloud2_ JSONToPointCloud2_(byte[] data) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        return mapper.readValue(data, PointCloud2_.class);
    }

    public static PoseArray_ JSONToPoseArray_(byte[] data) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        return mapper.readValue(data, PoseArray_.class);
    }

    public static BaseConfig_ JSONToStartGoal_(byte[] data) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        return mapper.readValue(data, BaseConfig_.class);
    }

    public static Agent JSONToAgent(byte[] data) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        return mapper.readValue(data, Agent.class);
    }

    public static class StartGoalSerializer {
        private static final Kryo kryo = new Kryo();

        public StartGoalSerializer() {
            MapSerializer serializer = new MapSerializer();
            kryo.register(HashMap.class, serializer);
            serializer.setKeyClass(String.class, serializer);
            serializer.setValueClass(PoseStamped_.class, serializer);

        }

        public static byte[] serialize(Object object) {

            ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
            Output output = new Output(byteArrayOutputStream);
            kryo.writeObject(output, object);
            output.flush();
            return byteArrayOutputStream.toByteArray();
        }

        public static Object deSerialize(byte[] b, Class e) {
            return kryo.readObject(new Input(b), e);
        }

    }

    /**
     * Serialize an object using kryo and return the bytes
     *
     * @param kryo   instance of kryo
     * @param object the object to be serialized
     * @return the serialized bytes
     */
    public static byte[] serialize(Kryo kryo, Object object) {
        ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
        Output output = new Output(byteArrayOutputStream);
//        output.setOutputStream(byteArrayOutputStream);
        kryo.writeObject(output, object);
        output.flush();
        return byteArrayOutputStream.toByteArray();
    }

    /**
     * De Serialize bytes using kryo and return the object
     *
     * @param kryo instance of kryo
     * @param b    the byte to be de serialized
     * @return the serialized bytes
     */
    public static Object deSerialize(Kryo kryo, byte[] b, Class e) {
        return kryo.readObject(new Input(new ByteArrayInputStream(b)), e);
    }

}
