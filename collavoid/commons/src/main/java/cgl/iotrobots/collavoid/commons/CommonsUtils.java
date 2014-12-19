package cgl.iotrobots.collavoid.commons;

import com.fasterxml.jackson.databind.ObjectMapper;
import nav_msgs.Odometry;

import java.io.ByteArrayOutputStream;
import java.io.IOException;

/*Adopted from iotrobots/turtlebot/commons/ */

public class CommonsUtils {
    public static byte[] TwistToJSON(Twist_ twist) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, twist);
        return outputStream.toByteArray();
    }

    public static Twist_ JSONToTwist_(byte[] data) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        return mapper.readValue(data, Twist_.class);
    }

    public static Odometry_ JSONToOdometry_(byte[] data) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        return mapper.readValue(data, Odometry_.class);
    }
}
