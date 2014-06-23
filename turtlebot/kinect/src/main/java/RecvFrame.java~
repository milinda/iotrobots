import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.MessageProperties;
import com.rabbitmq.client.QueueingConsumer;

import org.openkinect.freenect.*;
import org.xerial.snappy.Snappy;
import com.jcraft.jzlib.*;

import java.io.*;
import java.lang.*;
import java.util.*;
import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

public class RecvFrame {

    //private static final String TASK_QUEUE_NAME = "frame_queue";
    private static final String EXCHANGE_NAME = "frames";

    public static void main(String[] args) throws InterruptedException, IOException {
	final ImageUI ui = new ImageUI();
	Thread t = new Thread(new Runnable() {
		@Override
		    public void run() {
		    ui.show();
		}
	    });
	t.start();
	
	ConnectionFactory factory = new ConnectionFactory();
	factory.setHost("localhost");
	Connection connection = factory.newConnection();
	Channel channel = connection.createChannel();
	
	//channel.queueDeclare(TASK_QUEUE_NAME, true, false, false, null);
	channel.exchangeDeclare(EXCHANGE_NAME, "fanout");
	String queueName = channel.queueDeclare().getQueue();
	channel.queueBind(queueName, EXCHANGE_NAME, "");	

	QueueingConsumer consumer = new QueueingConsumer(channel);
	channel.basicConsume(queueName, true, consumer);
	
	double t_gamma[] = new double[1024];
	for(int p=0; p<1024; p++) {
	    t_gamma[p]=100 * 0.1236 * Math.tan(p / 2842.5 + 1.1863);
	}
	
	BufferedImage im = new BufferedImage(640, 480, BufferedImage.TYPE_INT_RGB);
	ByteBuffer frame = ByteBuffer.allocate(614400);	
	QueueingConsumer.Delivery delivery = null;
	System.out.println("Waiting for delivery");

        do {
	    delivery = consumer.nextDelivery(); //stops here...
	    //channel.basicAck(delivery.getEnvelope().getDeliveryTag(), false);
	    int err;
	    byte[] data = delivery.getBody();
	    byte[] restored = new byte[614400];

	    Inflater inflater = new Inflater();
	    
	    inflater.setInput(data);
		
	    while(true){
		inflater.setOutput(restored);
		err=inflater.inflate(JZlib.Z_NO_FLUSH);
		if(err==JZlib.Z_STREAM_END) break;
		CHECK_ERR(inflater, err, "inflate large");
	    }
	    
	    err=inflater.end();
	    CHECK_ERR(inflater, err, "inflateEnd");

	    for(int i=0; i<614400; i++) frame.put(i, restored[i]);  

	    int r,b,g,x,y;
	    for(int i=0; i<614400; i+=2) {		    
		int lo = frame.get(i) & 0xFF;
		int hi = frame.get(i+1) & 0xFF;
		int disp = hi << 8 | lo;
		double dist = t_gamma[disp];
		if (dist >= 40 && dist<150) {
		    b = 255;
		    r = 0;
		    g = (int)(255-((dist-40)/109*255));
		} else if (dist >= 150 && dist <= 250) {
		    dist = ((dist-150)/100*255);
		    b = (int)(255-dist);
		    r = (int)(dist);
		    g = 0;
		} else if (dist > 250 && dist <= 500){
		    dist = (dist-251)/249*255;
		    b = 0;
		    r = (int)(255-dist);
		    g = (int)(dist);
		} else if (disp==1023){
		    b = 0;
		    r = 0;
		    g = 0;
		} else {
		    dist = (dist-501)/t_gamma[1022]*255;
		    b = 20;
		    r = 0;
		    g = (int)(255-dist);
		}
		
		y=(int)Math.floor((double)i/2/640);
		x=i/2-640*y;
		int pixel = (0xFF) << 24
		    | (b & 0xFF) << 16
		    | (g & 0xFF) << 8
		    | (r & 0xFF) << 0;
		im.setRGB(x, y, pixel);
	    } 
		ui.setImage(im);
		ui.repaint();
	} while (delivery!=null);

	System.out.println("exitting");

	channel.close();
	connection.close();
    }

    static void CHECK_ERR(ZStream z, int err, String msg) {
	if(err!=JZlib.Z_OK){
	    if(z.msg!=null) System.out.print(z.msg+" "); 
	    System.out.println(msg+" error: "+err); 
	    
	    System.exit(1);
	}
    }
}


