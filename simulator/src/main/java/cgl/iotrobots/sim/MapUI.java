package cgl.iotrobots.sim;

import cgl.iotrobots.slam.core.app.GFSMap;
import cgl.iotrobots.slam.core.app.GFSAlgorithm;
import cgl.iotrobots.slam.core.utils.IntPoint;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.Random;

public class MapUI extends JFrame {
    GFSMap map;

    private ImagePanel im = new ImagePanel();

    public MapUI() throws HeadlessException {
        setTitle("MAP UI");
        setSize(800, 800);
        setResizable(true);
        add(im);
        setVisible(true);
    }

    public void setMap(GFSMap map) {
        this.map = map;
        im.setMap();
    }

    private class ImagePanel extends JPanel {

        public void setMap() {
            repaint();
        }

        @Override
        protected void paintComponent(Graphics g) {
            // super.paintComponent(g);

            if (map == null) {
                return;
            }

            if (map.width <= 0 || map.height <= 0) {
                return;
            }

            BufferedImage image = new BufferedImage(map.width, map.height, BufferedImage.TYPE_INT_ARGB);
            // System.out.println(map.width + " ," + map.height);

            for (int x = 0; x < map.width; x++) {
                for (int y = 0; y < map.height; y++) {
                    image.setRGB(x, y, Color.BLACK.getRGB());
                }
            }
            int count = 0;
            try {
                synchronized (map.currentPos) {
                    for (IntPoint p : map.currentPos) {
                        colorArea(image, p.x, p.y, map.width, map.height, 1);
                    }
                }
            } catch (Throwable e) {
                e.printStackTrace();
            }

            for (int x = 0; x < map.width; x++) {
                for (int y = 0; y < map.height; y++) {
                    int mapX = x;
                    int mapY = map.height - y - 1;
                    int occ = map.data[GFSAlgorithm.MAP_IDX(map.width, mapX, mapY)];
                    if (occ == 100) {
                        count++;
                        colorArea(image, x, y, map.width, map.height, 1);
                    }
                }
            }

            try {
                image = getScaledImage(image, 800, 800);
            } catch (IOException e) {
                e.printStackTrace();
            }

            g.drawImage(image, 0, 0, null);
            repaint();
        }
    }

    public static void colorArea(BufferedImage image, int x, int y, int w, int h, int size) {
        for (int i = -size; i < size; i++) {
            for (int j = -size; j < size; j++) {
                if (x + i < w && x + i > 0 && y + j < h && y + j > 0) {
                    image.setRGB(x + i, y + j, Color.WHITE.getRGB());
                }
            }
        }
    }

    public static BufferedImage getScaledImage(BufferedImage image, int width, int height) throws IOException {
        int imageWidth  = image.getWidth();
        int imageHeight = image.getHeight();

        double scaleX = (double)width/imageWidth;
        double scaleY = (double)height/imageHeight;
        AffineTransform scaleTransform = AffineTransform.getScaleInstance(scaleX, scaleY);
        AffineTransformOp bilinearScaleOp = new AffineTransformOp(scaleTransform, AffineTransformOp.TYPE_BILINEAR);

        return bilinearScaleOp.filter(
                image,
                new BufferedImage(width, height, image.getType()));
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
