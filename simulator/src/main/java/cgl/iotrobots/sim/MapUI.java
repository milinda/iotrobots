package cgl.iotrobots.sim;

import cgl.iotrobots.slam.core.sample.OutMap;
import cgl.iotrobots.slam.core.sample.Sample;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;

public class MapUI extends JFrame {
    int w = 600;
    int h = 600;

    BufferedImage image = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);

    OutMap map;

    private ImagePanel im = new ImagePanel();

    public MapUI() throws HeadlessException {
        ImagePanel panel = new ImagePanel();
        setTitle("AAA");
        setSize(600, 600);
        setResizable(false);
        add(im);
        setVisible(true);
    }

    public void setMap(OutMap map) {
        this.map = map;
        im.setMap();
    }

    private class ImagePanel extends JPanel {

        public void setMap() {
            repaint();
        }

        @Override
        protected void paintComponent(Graphics g) {
            if (map == null) {
                super.paintComponent(g);
                return;
            }

            for (int x = 0; x < map.width; x++) {
                for (int y = 0; y < map.height; y++) {
                    int occ = map.data[Sample.MAP_IDX(map.width, x, y)];
                    if (occ == -1) {
                        image.setRGB(x, y, Color.BLACK.getRGB());
                    } else if (occ == 100) {
                        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
//                    System.out.print("x");
                        image.setRGB(x, y, Color.WHITE.getRGB());
                    } else {
                        image.setRGB(x, y, Color.GRAY.getRGB());
                    }

                }
                //System.out.print("\n");
            }

            g.drawImage(image, 0, 0, null);
        }
    }
}
