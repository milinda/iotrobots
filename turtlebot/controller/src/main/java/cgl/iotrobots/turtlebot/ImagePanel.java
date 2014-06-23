package cgl.iotrobots.turtlebot;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;

public class ImagePanel extends JPanel {
    BufferedImage image;

    public void setImage(BufferedImage image) {
        this.image = image;
    }

    public void paint(Graphics g) {
        g.drawImage(image, 0, 0, null);
        repaint();
    }
}
