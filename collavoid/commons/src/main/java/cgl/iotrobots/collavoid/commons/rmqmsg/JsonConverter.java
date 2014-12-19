package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.IOException;

public class JsonConverter {

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
}
