package cgl.iotrobots.turtlebot;

import cgl.iotrobots.turtlebot.commons.Motion;
import cgl.iotrobots.turtlebot.commons.Velocity;

import java.awt.*;
import java.awt.event.KeyEvent;

public class EventHandler {
    private double increment = .1;

    private TurtleController controller;

    public EventHandler(TurtleController controller) {
        this.controller = controller;
    }

    public void handlerKeyPressed(KeyEvent e) {
        if (e.getKeyCode() == KeyEvent.VK_UP) {
            System.out.println("UP");
            controller.addVelocity(new Motion(new Velocity(increment, 0, 0), new Velocity(0, 0, 0)));
        } else if (e.getKeyCode() == KeyEvent.VK_DOWN) {
            System.out.println("DOWN");
            controller.addVelocity(new Motion(new Velocity(-increment, 0, 0), new Velocity(0, 0, 0)));
        } else if (e.getKeyCode() == KeyEvent.VK_LEFT) {
            System.out.println("LEFT");
            controller.addVelocity(new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, 1)));
        } else if (e.getKeyCode() == KeyEvent.VK_RIGHT) {
            System.out.println("RIGHT");
            controller.addVelocity(new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, -1)));
        }
    }

    public void setIncrement(double increment) {
        this.increment = increment;
    }
}
