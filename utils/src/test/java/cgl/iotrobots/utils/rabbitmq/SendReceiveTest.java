package cgl.iotrobots.utils.rabbitmq;

import junit.framework.TestCase;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

public class SendReceiveTest extends TestCase {
    RabbitMQSender sender;
    RabbitMQReceiver receiver1;
    RabbitMQReceiver receiver2;
    @Override
    protected void setUp() throws Exception {
        super.setUp();

        sender = new RabbitMQSender("amqp://localhost:5672", "slam_broadcast", true);
        receiver1 = new RabbitMQReceiver("amqp://localhost:5672", "slam_broadcast", true);
        receiver2 = new RabbitMQReceiver("amqp://localhost:5672", "slam_broadcast", true);

        sender.open();
    }

    public void testSendReceive() throws Exception {
        MessageReceiver r1 = new MessageReceiver(1);
        MessageReceiver r2 = new MessageReceiver(2);
        receiver1.listen(r1);
        //receiver1.listen(r);
        receiver2.listen(r2);

        Message message = new Message("Hello".getBytes(), new HashMap<String, Object>());

        for (int i = 0; i < 1000; i++) {
            sender.send(message, "");
            Thread.sleep(10);
        }

        while (r1.getCount() < 1000 && r2.getCount() < 1000) {
            Thread.sleep(100);
        }
    }

    public class MessageReceiver implements MessageHandler {
        AtomicInteger count = new AtomicInteger();
        int n;

        public MessageReceiver(int n) {
            this.n = n;
        }

        @Override
        public Map<String, String> getProperties() {
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_QUEUE, "pa" + "_" + n);
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, "*");
            return props;
        }

        @Override
        public void onMessage(Message message) {
            count.getAndIncrement();
            System.out.println(n + " : " + count.get());
        }

        public int getCount() {
            return count.get();
        }
    }

    public void testMap() throws Exception {
        RabbitMQReceiver receiver = new RabbitMQReceiver("amqp://localhost:5672", "slam_direct", false);
        receiver.listen(new MessageHandler() {
            int count = 0;
            @Override
            public Map<String, String> getProperties() {
                Map<String, String> props = new HashMap<String, String>();
                props.put(MessagingConstants.RABBIT_QUEUE, "test.test.map");
                props.put(MessagingConstants.RABBIT_ROUTING_KEY, "test.test.map");
                return props;
            }

            @Override
            public void onMessage(Message message) {
                System.out.println("Received" + count++);
            }
        });
        Thread.sleep(1000000);
    }
}
