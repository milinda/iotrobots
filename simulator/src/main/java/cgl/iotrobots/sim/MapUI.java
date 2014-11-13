package cgl.iotrobots.sim;

import cgl.iotrobots.slam.core.sample.OutMap;
import cgl.iotrobots.slam.core.sample.Sample;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.Random;

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
            // super.paintComponent(g);

            if (map == null) {
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

            for (int x = 0; x < map.width; x++) {
                for (int y = 0; y < map.height; y++) {
                    int occ = map.data[Sample.MAP_IDX(map.width, x, y)];

                    Random rand = new Random();
                    float r = rand.nextFloat();
                    float gg = rand.nextFloat();
                    float b = rand.nextFloat();
                    Color randomColor = new Color(r, gg, b);
                    // image.setRGB(x, y, randomColor.getRGB());
                    if (occ == 100) {
                        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
                        count++;

                        // image.setRGB(x, y, Color.WHITE.getRGB());
                        colorArea(image, x, y, map.width, map.height);
                    }
                }
            }
            // System.out.println("One scan completed: " + count);


//            BufferedImage r = scale(image, BufferedImage.TYPE_INT_ARGB, w, h, map.width, map.height);
            try {
                image = getScaledImage(image, 600, 600);
            } catch (IOException e) {
                e.printStackTrace();
            }

            g.drawImage(image, 0, 0, null);
            repaint();
        }
    }

    public static void colorArea(BufferedImage image, int x, int y, int w, int h) {
        int size = 4;
//        System.out.println("x, y" + x + ", " + y);
        for (int i = -size; i < size; i++) {
            for (int j = -size; j < size; j++) {
                if (x + i < w && x + i > 0 && y + j < h && y + j > 0) {
                    Random rand = new Random();
                    float r = rand.nextFloat();
                    float gg = rand.nextFloat();
                    float b = rand.nextFloat();
                    Color randomColor = new Color(r, gg, b);
                    image.setRGB(x + i, y + j, Color.WHITE.getRGB());
//                    image.setRGB(x + i, y + j, randomColor.getRGB());
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
