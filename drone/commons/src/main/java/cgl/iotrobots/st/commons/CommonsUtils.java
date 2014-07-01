package cgl.iotrobots.st.commons;

import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;

public class CommonsUtils {
    public static byte[] motionToJSON(JSONControlMessage motion) throws IOException {
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, motion);
        return outputStream.toByteArray();
    }

    public static JSONControlMessage jsonToMotion(byte []data) throws IOException {
        ObjectMapper mapper = new ObjectMapper();   // can reuse, share globally
        return mapper.readValue(data, JSONControlMessage.class);
    }

    public static void main(String[] args) throws IOException {
        JSONControlMessage message = new JSONControlMessage();
        Control control = new Control(new double[]{1.0, 2.0}, false);
        message.setControl(control);
        System.out.println(new String(motionToJSON(message)));
    }
}
