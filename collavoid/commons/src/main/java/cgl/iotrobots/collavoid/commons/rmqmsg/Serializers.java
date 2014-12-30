package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;
import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;

public class Serializers {
    private static final Kryo kryo = new Kryo();

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

    public static byte[] serialize(Object object) {
        ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
        Output output = new Output(byteArrayOutputStream);
        kryo.writeObject(output, object);
        output.flush();
        return byteArrayOutputStream.toByteArray();
    }

    public static Object deSerialize(byte[] b, Class e) {
        return kryo.readObject(new Input(new ByteArrayInputStream(b)), e);
    }
}
