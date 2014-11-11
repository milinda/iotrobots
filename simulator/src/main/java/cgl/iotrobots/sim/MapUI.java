package cgl.iotrobots.sim;

import cgl.iotrobots.slam.core.sample.OutMap;
import cgl.iotrobots.slam.core.sample.Sample;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;

public class MapUI extends JFrame {
    int w = 200;
    int h = 200;



    OutMap map;

    private ImagePanel im = new ImagePanel();

    public MapUI() throws HeadlessException {
        ImagePanel panel = new ImagePanel();
        setTitle("AAA");
        setSize(800, 800);
        setResizable(true);
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

            BufferedImage image = new BufferedImage(map.width, map.height, BufferedImage.TYPE_INT_ARGB);

            for (int x = 0; x < map.width; x++) {
                int count = 0;
                for (int y = 0; y < map.height; y++) {
                    int occ = map.data[Sample.MAP_IDX(map.width, x, y)];
                    if (occ == -1) {
                        image.setRGB(x, y, Color.BLACK.getRGB());
                        //System.out.println("0");
                    } else if (occ == 100) {
                        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
                        count++;
                        image.setRGB(x, y, Color.WHITE.getRGB());
                    } else {
                        image.setRGB(x, y, Color.YELLOW.getRGB());
                       // System.out.println("y");
                    }

                }
//                System.out.println("One scan completed: " + count);
            }

            BufferedImage r = scale(image, BufferedImage.TYPE_INT_ARGB, w, h, map.width, map.height);

            g.drawImage(r, 0, 0, null);
        }


    }

    public static BufferedImage scale(BufferedImage sbi, int imageType, int dWidth, int dHeight, double fWidth, double fHeight) {
        BufferedImage dbi = null;
        if(sbi != null) {
            dbi = new BufferedImage(dWidth, dHeight, imageType);
            Graphics2D g = dbi.createGraphics();
            AffineTransform at = AffineTransform.getScaleInstance(fWidth, fHeight);
            g.drawRenderedImage(sbi, at);
        }
        return dbi;
    }
}
