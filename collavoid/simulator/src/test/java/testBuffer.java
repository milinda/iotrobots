import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;

import java.nio.ByteOrder;

/**
 * Created by hjh on 12/1/14.
 */
public class testBuffer {

    public static void main(String[] args){
        ChannelBuffer buffer= ChannelBuffers.buffer(ByteOrder.LITTLE_ENDIAN,500);
        for (int i = 0; i <10 ; i++) {
            System.out.println("readable: "+buffer.readableBytes());
            System.out.println("writable: "+buffer.writableBytes());
            float value=(float)Math.random();
            System.out.println(value);
            buffer.writeFloat(value);
        }
        System.out.println("------------------");
        for (int i = 0; i <10 ; i++) {
            System.out.println("readable: "+buffer.readableBytes());
            System.out.println("writable: "+buffer.writableBytes());
            System.out.println(buffer.readFloat());
            buffer.discardReadBytes();
        }

    }
}
