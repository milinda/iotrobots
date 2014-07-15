/**
 (c) 2009-2020 by Columbia University; all rights reserved

 Permission to use, copy, modify, and distribute this software and its
 documentation for research and educational purpose and without fee is
 hereby granted, provided that the above copyright notice appear in all
 copies and that both that the copyright notice and warranty disclaimer
 appear in supporting documentation, and that the names of the copyright
 holders or any of their entities not be used in advertising or publicity
 pertaining to distribution of the software without specific, written
 prior permission.  Use of this software in whole or in parts for direct
 commercial advantage requires explicit prior permission.

 The copyright holders disclaim all warranties with regard to this
 software, including all implied warranties of merchantability and
 fitness.  In no event shall the copyright holders be liable for any
 special, indirect or consequential damages or any damages whatsoever
 resulting from loss of use, data or profits, whether in an action of
 contract, negligence or other tortuous action, arising out of or in
 connection with the use or performance of this software.
 */

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.awt.image.*;

import java.io.*;

import java.util.*;

import javax.imageio.*;
import javax.imageio.stream.*;

import javax.swing.*;


public class TimeRenderer
        extends JFrame
        implements Runnable {

    JPanel panel;
    int x;
    int y;
    BufferedImage[] barcodes = new BufferedImage[10000];
    BufferedImage[] barcodes2 = new BufferedImage[10000];
    boolean isBarcode = false;
    int refreshTime = 1;
    int barcodeNum = 10000;
    boolean timingPrint = false;
    int size = 1;
    boolean isPre = false;
    TimerTask myTimerTask;
    java.util.Timer timer;
    BufferedImage biCrop2;
    String strFontName2 = "Ariel";
    int nFontSize2 = 72;
    Font font2 = new Font(strFontName2, 0, nFontSize2);

    public TimeRenderer(String[] str) {
        setTitle("Hello 2D");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setAlwaysOnTop(true);
        x = Integer.parseInt(str[0]);
        y = Integer.parseInt(str[1]);

        for (int x = 2; x < str.length; x++) {
            if (str[x].equalsIgnoreCase("-barcode"))
                isBarcode = true;
            else if (str[x].equalsIgnoreCase("-time"))
                refreshTime = Integer.parseInt(str[++x]);
            else if (str[x].equalsIgnoreCase("-preprocessed"))
                isPre = true;
        }

        System.out.println("Barcode:" + isBarcode);
        System.out.println("Refresh Time:" + refreshTime);
        System.out.println("Preprocessed Barcode:" + isPre);

        if (isBarcode) {

            int number;
            String filename;
            long beforeReading = System.nanoTime();
            System.out.print("Reading barcodes...");
            System.out.flush();

            if (isPre) {
                try {
                    DataInputStream fis = new DataInputStream(new BufferedInputStream(new FileInputStream(
                            "barcodes.data")));
                    byte[] data = new byte[30000];
                    ImageReader reader = (ImageReader) (ImageIO.getImageReadersByFormatName(
                            "png").next());

                    for (int x = 0; x < barcodeNum; x++) {
                        int len = fis.readInt();
                        fis.readFully(data, 0, len);
                        ByteArrayInputStream bais = new ByteArrayInputStream(
                                data, 0, len);
                        ImageInputStream iis = ImageIO.createImageInputStream(
                                bais);
                        reader.setInput(iis, true);
                        //long time = System.nanoTime();
                        barcodes[x] = loadCompatibleImage(reader.read(0));
                        //time = System.nanoTime()-time;
                        //System.out.println(" "+time);
                    }

                    fis.close();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            } else {
                DataOutputStream fos = null;
                try {
                    fos = new DataOutputStream(new FileOutputStream(
                            "barcodes.data"));
                    //oos.writeObject(barcodes);
                    //oos.close();
                } catch (Exception e) {
                    e.printStackTrace();
                }

                for (int x = 0; x < barcodeNum; x++) {
                    try {

                        if (x % 100 == 0) {
                            System.out.print(
                                    " " + x + "-" +
                                            ((System.nanoTime() - beforeReading) / 1000000000) +
                                            "sec");

                            //System.gc();
                        }

                        number = barcodeNum + x;
                        filename = number + ".png";
                        barcodes[x] = ImageIO.read(new File("barcodes\\" + filename.substring(
                                1,
                                filename.length()))); //loadCompatibleImage(new File("barcodes\\"+filename.substring(1,filename.length())));

                        BufferedImage biCrop = new BufferedImage(barcodes[x].getWidth(),
                                2,
                                barcodes[x].getType());

                        //BufferedImage biCrop2 = new BufferedImage(barcodes[x].getWidth(), 1,barcodes[x].getType());
                        Graphics2D g2d = biCrop.createGraphics();

                        //Graphics2D g2d = biCrop2.createGraphics();
                        BufferedImage bi2 = barcodes[x].getSubimage(0, 142,
                                barcodes[x].getWidth(),
                                1);
                        g2d.drawImage(bi2, null, 0, 0);

                        BufferedImage bi3 = barcodes[x].getSubimage(0, 144,
                                barcodes[x].getWidth(),
                                1);
                        g2d.drawImage(bi3, null, 0, 1);
                        g2d.dispose();
                        barcodes[x] = loadCompatibleImage(biCrop);

                        ByteArrayOutputStream baos = new ByteArrayOutputStream();
                        ImageIO.write(barcodes[x], "png", baos);
                        fos.writeInt(baos.toByteArray().length);
                        fos.write(baos.toByteArray(), 0,
                                baos.toByteArray().length);

                        //barcodes2[x]=biCrop2;
                        //ImageIO.read(new File(filename.substring(1,filename.length())));
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }

                try {

                    //new FileOutputStream("barcodes.data");
                    //oos.writeObject(barcodes);
                    fos.close();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            System.out.println(
                    "done in " +
                            ((System.nanoTime() - beforeReading) / 1000000000) +
                            "sec");
            System.gc();
        }

        init();
        setLocation(x, y);
        setUndecorated(true);
        if (isBarcode) setSize(new Dimension(barcodes[0].getWidth(), (161 + 30)));
        else setSize(new Dimension(280, 70));
        //pack();
        setVisible(true);
        this.setIgnoreRepaint(true);
        
        /*
        myTimerTask = new MyTimerTask();
        timer = new java.util.Timer();
	timer.scheduleAtFixedRate(myTimerTask, 0, refreshTime);
        */
        /*
        GraphicsEnvironment environment = GraphicsEnvironment
        .getLocalGraphicsEnvironment();
        GraphicsDevice device = environment.getDefaultScreenDevice();
         device.setFullScreenWindow(this);
        //DisplayMode dm = new DisplayMode(640, 400, 32, 0);
         DisplayMode dm = new DisplayMode(1280, 800, 32, 60);
         device.setDisplayMode(dm);
         System.out.println( device.getDisplayMode().getWidth()+","+
                            device.getDisplayMode().getHeight()+","+
                            device.getDisplayMode().getBitDepth()+","+
                            device.getDisplayMode().getRefreshRate()+"");
        */


        createBufferStrategy(2);


        Thread s = new Thread(this);
        s.start();
        new CommandLine().start();
    }

    class MyTimerTask
            extends TimerTask {
        public void run() {

            try {

                //Thread.sleep(refreshTime);
                //System.out.print("1."+System.nanoTime()/1000000);
                panel.paint(panel.getGraphics());

                //System.out.println(" 3."+System.nanoTime()/1000000);
            } catch (Exception e) {
            }
        }
    }

    class CommandLine
            extends Thread {

        boolean isContinue = true;
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));

        public void run() {
            System.out.println("Available Commands");
            System.out.println("quit|exit");
            System.out.println("time X (default:10 in terms of ms)");
            System.out.println("size X (default:3 between 1 and 6)");
            System.out.println("position X X (first is x and second is y)");

            while (isContinue) {

                StringTokenizer st = null;

                try {
                    st = new StringTokenizer(br.readLine());
                } catch (Exception e) {
                }

                String command = st.nextToken();

                if (command.equalsIgnoreCase("quit") ||
                        command.equalsIgnoreCase("exit"))
                    isContinue = false;
                else if (command.equalsIgnoreCase("time")) {
                    refreshTime = Integer.parseInt(st.nextToken());
                    timer.cancel();
                    myTimerTask = new MyTimerTask();
                    timer = new java.util.Timer();
                    timer.scheduleAtFixedRate(myTimerTask, 0, refreshTime);
                } else if (command.equalsIgnoreCase("size")) {
                    size = Integer.parseInt(st.nextToken()) - 1;
                    size = (size % 6) + 1;
                    setPreferredSize(new Dimension(339 * size / 3,
                            (161 + 30) * size / 3));
                    pack();
                } else if (command.equalsIgnoreCase("position")) {
                    x = Integer.parseInt(st.nextToken());
                    y = Integer.parseInt(st.nextToken());
                    setLocation(x, y);
                }
            }

            System.exit(0);
        }
    }

    public static BufferedImage loadCompatibleImage(File resource)
            throws IOException {

        BufferedImage image = ImageIO.read(resource);

        return loadCompatibleImage(image);
    }

    public static BufferedImage loadCompatibleImage(BufferedImage image)
            throws IOException {

        //if (true) return image;
        GraphicsConfiguration configuration = GraphicsEnvironment.getLocalGraphicsEnvironment()
                .getDefaultScreenDevice().getDefaultConfiguration();
        BufferedImage compatibleImage = configuration.createCompatibleImage(image.getWidth(),
                image.getHeight());
        Graphics g = compatibleImage.getGraphics();
        g.drawImage(image, 0, 0, null);
        g.dispose();

        return compatibleImage;
    }

    public void draw() {
        BufferStrategy bf = this.getBufferStrategy();
        Graphics g = bf.getDrawGraphics();
        g.setColor(Color.white);
        g.fillRect(0, 0, this.getWidth(), this.getHeight());
        long time;

        if (isBarcode) {
            //timex = System.currentTimeMillis();
            //if (timex%10!=0)try{Thread.sleep(10-(timex%10));}catch(Exception e){}
            //g2.drawImage(tr.barcodes[(int)((System.currentTimeMillis()/10)%1000)], 0, 10, null);
            int pos = (int) ((System.currentTimeMillis()) % barcodeNum);

            //long start=System.nanoTime();
            int x;

            for (x = 0; x < 144; x++)
                g.drawImage(barcodes[pos], 0, 15 + 1 * 3 / 3 * x, 339 * 3 / 3,
                        15 + 1 + 1 * 3 / 3 * x, 0, 0, 339, 1, null);

            //g2d.drawImage(tr.barcodes[pos], 0, 1*3/3*x,339*3/3,1+1*3/3*x,0, 1,339,2, null);
            //x++;
            for (; x < 161; x++)
                g.drawImage(barcodes[pos], 0, 15 + 1 * 3 / 3 * x, 339 * 3 / 3,
                        15 + 1 + 1 * 3 / 3 * x, 0, 1, 339, 2, null);

            //g2d.dispose();
            //for (int x=0;x<150;x++)
            //g2.drawImage(tr.barcodes[pos], 0, 10+1*3/3*x,339*3/3,10+1+1*3/3*x,0, 0,339,1, null);
            //g2.drawImage(tr.barcodes[pos], 0, 10,339*tr.size/3,10+177*tr.size/3,0, 0,339,177, null);

            //g2.drawImage(biCrop, 0, 15, biCrop.getWidth() * size,
            //             15 + biCrop.getHeight() * size, 0, 0,
            //             biCrop.getWidth(), biCrop.getHeight(), null);

            //g2.drawImage(biCrop, 0, 10 , null);
            //g2.drawImage(tr.barcodes[pos], 0, 10 , null);
            //start = System.nanoTime()-start;
            //System.out.println(pos+" "+start+"nano");
        } else {
            time = System.nanoTime() / 1000000;
            g.setColor(Color.black);
            g.setFont(font2);
            g.drawString((time / 1000) % 1000 + "." + time % 1000, 5, 64);
            //System.out.print(" 2." + System.nanoTime() / 1000000);
        }
        g.dispose();
        bf.show();
        //Toolkit.getDefaultToolkit().sync();

    }

    public void run() {
        long time = System.currentTimeMillis();
        int fps = 0;
        long startTime, drawTime, lastTime = System.nanoTime();
        int counter = 0;
        try {

            while (true) {
                startTime = System.nanoTime();

                //YOUR UPDATES

                draw();
                counter++;
                if (System.nanoTime() - lastTime > 1000000000) {
                    fps = counter;
                    //System.out.println(fps+"");
                    lastTime = System.nanoTime();
                    counter = 0;
                }
                drawTime = ((System.nanoTime() - startTime) / 1000000);
                try {
                    if (drawTime > 16) ;// Thread.sleep(1);
                    else Thread.sleep(16 - drawTime);
                } catch (Exception e) {
                }

            }
        } catch (Exception e) {
        }
    }

    public static void main(String[] s) {

        TimeRenderer frame = new TimeRenderer(s);
    }

    public void init() {
        panel = new Hello2DPanel(this);
        panel.addMouseListener(new MouseAdapter() {
            public void mouseClicked(MouseEvent e) {
                size = (size % 4) + 1;
                setPreferredSize(new Dimension(339 * size, (161 + 30) * size));
                pack();
            }
        });
        getContentPane().add(panel);
    }
}

class Hello2DPanel
        extends JPanel {

    long time = 0;
    String strFontName = "Ariel";
    int nFontSize = 72;
    Font font;
    TimeRenderer tr;
    int counter = 0;
    long timex;
    BufferedImage biCrop;
    Graphics2D g2d;

    public Hello2DPanel(TimeRenderer trx) {
        tr = trx;

        GraphicsConfiguration configuration = GraphicsEnvironment.getLocalGraphicsEnvironment()
                .getDefaultScreenDevice().getDefaultConfiguration();
        if (tr.isBarcode) {
            biCrop = configuration.createCompatibleImage(tr.barcodes[0].getWidth(),
                    161);

            //biCrop = new BufferedImage(tr.barcodes[0].getWidth(), 170,tr.barcodes[0].getType());
            g2d = biCrop.createGraphics();
            setPreferredSize(new Dimension(tr.barcodes[0].getWidth(), (161 + 30)));
        } else {
            setPreferredSize(new Dimension(280, 70));
        }
        font = new Font(strFontName, 0, nFontSize);
        setBackground(Color.white);
    }

    public void paintComponent(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2 = (Graphics2D) g;
        g2.setColor(Color.black);
        g2.setFont(font);

        //Ellipse2D e = new Ellipse2D.Double(-100, -50, 200, 100);
        //AffineTransform tr = new AffineTransform();
        //tr.rotate(Math.PI / 6.0);
        //Shape shape = tr.createTransformedShape(e);
        //g2.translate(300, 200);
        //g2.scale(2, 2);
        //g2.draw(shape);
        //time = System.currentTimeMillis();
        if (tr.isBarcode) {

            //timex = System.currentTimeMillis();
            //if (timex%10!=0)try{Thread.sleep(10-(timex%10));}catch(Exception e){}
            //g2.drawImage(tr.barcodes[(int)((System.currentTimeMillis()/10)%1000)], 0, 10, null);
            int pos = (int) ((System.currentTimeMillis()) % tr.barcodeNum);

            //long start=System.nanoTime();
            int x;

            for (x = 0; x < 144; x++)
                g2d.drawImage(tr.barcodes[pos], 0, 1 * 3 / 3 * x, 339 * 3 / 3,
                        1 + 1 * 3 / 3 * x, 0, 0, 339, 1, null);

            //g2d.drawImage(tr.barcodes[pos], 0, 1*3/3*x,339*3/3,1+1*3/3*x,0, 1,339,2, null);
            //x++;
            for (; x < 161; x++)
                g2d.drawImage(tr.barcodes[pos], 0, 1 * 3 / 3 * x, 339 * 3 / 3,
                        1 + 1 * 3 / 3 * x, 0, 1, 339, 2, null);

            //g2d.dispose();
            //for (int x=0;x<150;x++)
            //g2.drawImage(tr.barcodes[pos], 0, 10+1*3/3*x,339*3/3,10+1+1*3/3*x,0, 0,339,1, null);
            //g2.drawImage(tr.barcodes[pos], 0, 10,339*tr.size/3,10+177*tr.size/3,0, 0,339,177, null);
            g2.drawImage(biCrop, 0, 15, biCrop.getWidth() * tr.size,
                    15 + biCrop.getHeight() * tr.size, 0, 0,
                    biCrop.getWidth(), biCrop.getHeight(), null);

            //g2.drawImage(biCrop, 0, 10 , null);
            //g2.drawImage(tr.barcodes[pos], 0, 10 , null);
            //start = System.nanoTime()-start;
            //System.out.println(pos+" "+start+"nano");
        } else {
            time = System.nanoTime() / 1000000;
            g2.drawString((time / 1000) % 1000 + "." + time % 1000, 5, 64);
            //System.out.print(" 2." + System.nanoTime() / 1000000);
        }
    }
}