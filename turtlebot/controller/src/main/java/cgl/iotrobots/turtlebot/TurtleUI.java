package cgl.iotrobots.turtlebot;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;

public class TurtleUI extends JFrame {
    private BufferedImage image = null;

    public TurtleUI() throws HeadlessException {
        setTitle("My Empty Frame");
        setSize(1204,768); // default size is 0,0
        setLocation(10, 200); // default is 0,0 (top left corner)

        // Add Panels
        Container contentPane = getContentPane();
        contentPane.add(new VideoPanel());
        JFrame f = new TurtleUI();
        f.setVisible(true);
    }

    class VideoPanel extends JPanel {
        public void paint(Graphics g) {
            g.drawImage(image, 0, 0, null);
            repaint();
        }
    }

    public void setImage(BufferedImage image) {
        this.image = image;
    }
}
