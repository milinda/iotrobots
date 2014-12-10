package cgl.iotrobots.slam.streaming;

import cgl.iotrobots.slam.streaming.msgs.ParticleAssignments;
import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;

/**
 * Utilities
 */
public class Utils {
    /**
     * Serialize an object using kryo and return the bytes
     * @param kryo instance of kryo
     * @param object the object to be serialized
     * @return the serialized bytes
     */
    public static byte[] serialize(Kryo kryo, Object object) {
        ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
        Output output = new Output();
        kryo.writeObject(output, object);
        output.flush();
        return byteArrayOutputStream.toByteArray();
    }

    /**
     * De Serialize bytes using kryo and return the object
     * @param kryo instance of kryo
     * @param b the byte to be de serialized
     * @return the serialized bytes
     */
    public static Object deSerialize(Kryo kryo, byte []b, Class e) {
        return kryo.readObject(new Input(new ByteArrayInputStream(b)), e);
    }
}
