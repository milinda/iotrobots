import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.MessageProperties;

import org.openkinect.freenect.*;
import org.xerial.snappy.Snappy;
import com.jcraft.jzlib.*;

import java.io.*;
import java.lang.*;
import java.util.*;
import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

public class SendFrame {

    //private static final String TASK_QUEUE_NAME = "frame_queue";
    private static final String EXCHANGE_NAME = "frames";

    public static void main(String[] args) throws InterruptedException {
	try {

	ConnectionFactory factory = new ConnectionFactory();
	factory.setHost("localhost");
	Connection connection = factory.newConnection();
	final Channel channel = connection.createChannel();

	//channel.queueDeclare(TASK_QUEUE_NAME, true, false, false, null);
	channel.exchangeDeclare(EXCHANGE_NAME, "fanout"); //stops here...	

        // DECLARATIONS
        Context ctx = null;
        Device dev = null;

        // INITIALIZE DEVICE
        ctx = Freenect.createContext();
        if (ctx.numDevices() > 0) {
            dev = ctx.openDevice(0);
        } else {
            System.err.println("No kinects detected.  Exiting.");
            System.exit(0);
        }
        // DISPLAY DEPTH VIDEO
	dev.startDepth(new DepthHandler() {
		int numFrame = 0;
		@Override
		    public void onFrameReceived(FrameMode mode, ByteBuffer frame, int timestamp) {
		    if(numFrame%2==0) {
			byte[] data = new byte[614400];
			for(int i=0; i<614399; i++) data[i] = frame.get(i);
			
			// compress data
			int err;
			int comprLen = 120000;
			int uncomprLen = 614400;
			byte[] uncompr=new byte[uncomprLen];
			byte[] compr=new byte[comprLen];
			
			Deflater deflater = null;
			try{
			    deflater = new Deflater(JZlib.Z_BEST_SPEED);
			}
			catch(GZIPException e){
			}
			
			deflater.setInput(data);
			deflater.setOutput(compr);
			
			err=deflater.deflate(JZlib.Z_NO_FLUSH);
			CHECK_ERR(deflater, err, "deflate");
			if(deflater.avail_in!=0){
			    System.out.println("deflate not greedy");
			    System.exit(1);
			}
			
			err=deflater.deflate(JZlib.Z_FINISH);
			if(err!=JZlib.Z_STREAM_END){
			    System.out.println("deflate should report Z_STREAM_END");
			    System.exit(1);
			}
			err=deflater.end();
			CHECK_ERR(deflater, err, "deflateEnd");
			try {
			    channel.basicPublish(EXCHANGE_NAME, "", null, compr);
			} catch (IOException e) {
			    System.exit(0);
			}

		    }
		    numFrame++;
		}
	    });

	Thread.sleep(100000);
	channel.basicPublish(EXCHANGE_NAME, "", null, null);
	//channel.basicPublish( "", _NAME, MessageProperties.PERSISTENT_TEXT_PLAIN, null);

	dev.stopDepth();

	channel.close();
	connection.close();
	
	// SHUT DOWN
	if (ctx != null) {
	    if (dev != null) {
		dev.close();
	    }
	}
	ctx.shutdown();
	} catch (IOException e) {
	    System.exit(0);
	}
    }

    static void CHECK_ERR(ZStream z, int err, String msg) {
	if(err!=JZlib.Z_OK){
	    if(z.msg!=null) System.out.print(z.msg+" "); 
	    System.out.println(msg+" error: "+err); 
	    
	    System.exit(1);
	}
    }
}

/*			byte[] compressed = null;
			try {
			    compressed = Snappy.compress(data);
			    
			    channel.basicPublish(EXCHANGE_NAME, "", null, compressed);
			} catch (IOException e) {
			    System.exit(0);
			} 
*/
