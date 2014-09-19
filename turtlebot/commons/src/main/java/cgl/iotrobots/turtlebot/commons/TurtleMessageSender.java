package cgl.iotrobots.turtlebot.commons;

import com.rabbitmq.client.AMQP;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;

import java.io.IOException;
import java.net.URISyntaxException;
import java.security.KeyManagementException;
import java.security.NoSuchAlgorithmException;
import java.util.HashMap;
import java.util.Map;

public class TurtleMessageSender {
    private String rountingKey;
    private Channel channel;

    private String exchange;

    public TurtleMessageSender(String url, String exchane, String routingKey) {
        ConnectionFactory factory = new ConnectionFactory();
        try {
            factory.setUri(url);
            final Connection connection = factory.newConnection();
            channel = connection.createChannel();
            channel.exchangeDeclare(exchane, "fanout", true);
            this.exchange = exchane;
            this.rountingKey = routingKey;
        } catch (URISyntaxException e) {
            e.printStackTrace();
        } catch (NoSuchAlgorithmException e) {
            e.printStackTrace();
        } catch (KeyManagementException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void send(byte body[], Map<String, Object> props) {
        try {
            channel.basicPublish(exchange, rountingKey, new AMQP.BasicProperties.Builder().headers(props).build(), body);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void close() {
        try {
            channel.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
