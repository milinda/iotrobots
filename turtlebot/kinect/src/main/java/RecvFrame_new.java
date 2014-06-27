import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.MessageProperties;
import com.rabbitmq.client.QueueingConsumer;

import org.openkinect.freenect.*;
import com.jcraft.jzlib.*;

import java.io.*;
import java.lang.*;
import java.util.*;
import java.awt.image.BufferedImage;

public class RecvFrame_new {

    public static void main(String[] args) throws InterruptedException, IOException {
	final ImageUI ui = new ImageUI();
	Thread t = new Thread(new Runnable() {
		@Override
		    public void run() {
		    ui.show();
		}
	    });
	t.start();
	
	// RABBITMQ DECLARATIONS
	ConnectionFactory factory = new ConnectionFactory();
	factory.setHost("localhost");
	final Connection connection = factory.newConnection();
	final Channel channel = connection.createChannel();
	String exchange_name = args[0];

	// BIND EXCHANGE TO QUEUE
	channel.exchangeDeclare(exchange_name, "fanout");
	String queueName = channel.queueDeclare().getQueue();
	channel.queueBind(queueName, exchange_name, "");	

	QueueingConsumer consumer = new QueueingConsumer(channel);
	channel.basicConsume(queueName, true, consumer);
	
	// CLEANLY SHUTDOWN
	Runtime.getRuntime().addShutdownHook(new Thread() {
		@Override
		    public void run() {
		    System.out.println("exitting");
		    try { 
			channel.close();
		    } catch (IOException e) {
			System.exit(0);
		    }
		}
	    });
	
	BufferedImage im = new BufferedImage(640, 480, BufferedImage.TYPE_INT_RGB);
	QueueingConsumer.Delivery delivery = null;
	System.out.println("Waiting for delivery");

        do {
	    // POP DATA OF QUEUE
	    delivery = consumer.nextDelivery(); 
	    int err;
	    byte[] data = delivery.getBody();
	    byte[] inverted = new byte[307200];

	    Inflater inflater = new Inflater();
	    
	    inflater.setInput(data);
		
	    // DECOMPRESS
	    while(true){
		inflater.setOutput(inverted);
		err=inflater.inflate(JZlib.Z_NO_FLUSH);
		if(err==JZlib.Z_STREAM_END) break;
		CHECK_ERR(inflater, err, "inflate large");
	    }
	    
	    err=inflater.end();
	    CHECK_ERR(inflater, err, "inflateEnd");

	    // COLOR DISTANCE DATA
	    int[] restored = new int[307200];
	    int r=0,b=0,g=0,x,y;
	    for(int i=0; i<307200; i++) {
		restored[i] = 0 | (inverted[i] & 0xFF);
		if(restored[i] == 0) {
		    b = 0;
		    r = 0;
		    g = 0;
		} else {
		    int dist = (int)(90300 / (restored[i] + 21.575));
		    if (dist >= 326 && dist < 1443) {
			b = 255;
			r = 0;
			g = 255; 
		    } else if (dist >= 1500 && dist <= 2500) {
			dist = ((dist-1500)/1000*255);
			b = 255;
			r = 0;
			g = 150;
		    } else if (dist > 2500 && dist <= 5000){
			dist = (dist-2501)/2499*255;
			b = 255;
			r = 0;
			g = 50;
		    }
		}
		
		y=(int)Math.floor((double)i/640);
		x=i-640*y;
		int pixel = (0xFF) << 24
		    | (b & 0xFF) << 16
		    | (g & 0xFF) << 8
		    | (r & 0xFF) << 0;
		im.setRGB(x, y, pixel);
            }
            ui.setImage(im);
            ui.repaint();
	} while (true);
    }

    static void CHECK_ERR(ZStream z, int err, String msg) {
	if(err!=JZlib.Z_OK){
	    if(z.msg!=null) System.out.print(z.msg+" "); 
	    System.out.println(msg+" error: "+err); 
	    
	    System.exit(1);
	}
    }
}


