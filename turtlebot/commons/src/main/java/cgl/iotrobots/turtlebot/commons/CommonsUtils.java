package cgl.iotrobots.turtlebot.commons;

import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;

public class CommonsUtils {
    public static byte[] motionToJSON(Motion motion) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, motion);
        return outputStream.toByteArray();
    }

    public static Motion jsonToMotion(byte []data) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        return mapper.readValue(data, Motion.class);
    }
}
