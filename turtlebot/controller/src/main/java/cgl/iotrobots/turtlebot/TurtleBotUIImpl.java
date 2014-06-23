package cgl.iotrobots.turtlebot;

import java.awt.event.KeyEvent;
import java.awt.image.BufferedImage;

public class TurtleBotUIImpl extends TurtlebotUI {
    private EventHandler eventHandler;

    public TurtleBotUIImpl(EventHandler eventHandler) {
        this.eventHandler = eventHandler;
    }

    @Override
    public void formKeyPressed(KeyEvent evt) {
        eventHandler.handlerKeyPressed(evt);
    }

    @Override
    public void jPanel1KeyPressed(KeyEvent evt) {
        eventHandler.handlerKeyPressed(evt);
    }

    public void setImage(BufferedImage image) {
        ImagePanel imagePanel = (ImagePanel) this.jPanel1;
        imagePanel.setImage(image);
        imagePanel.repaint();
    }
}
