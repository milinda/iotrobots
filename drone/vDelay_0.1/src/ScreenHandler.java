/**(c) 2009-2020 by Columbia University; all rights reserved

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
import java.awt.image.*;
import java.awt.peer.*;
import java.io.*;
import java.util.*;
import javax.imageio.*;

import sun.awt.SunToolkit;
//import com.sun.media.imageio.plugins.tiff.TIFFImageWriteParam;
import javax.imageio.stream.ImageOutputStream;
import javax.imageio.ImageWriteParam;


import com.google.zxing.DecodeHintType;
import com.google.zxing.MonochromeBitmapSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.ReaderException;
import com.google.zxing.Result;
import com.google.zxing.client.result.ParsedResult;
import com.google.zxing.client.result.ResultParser;
import com.google.zxing.oned.*;
import com.google.zxing.client.j2se.*;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.charset.Charset;
import java.util.Hashtable;

import javax.swing.*;

public class ScreenHandler
        implements Runnable {

    int x;
    int y;
    int w;
    int h;
    BufferedImage oldBI = null;
    BufferedImage newBI = null;
    int[] oldArray;
    int[] newArray;
    RobotPeer robotPeer;
    Rectangle r;
    boolean isRobotPeer = false;
    private static Robot nano;

    boolean isDecodeBarcode = true;
    int cursorTime = 5000;
    String filename = "stats";
    boolean isTryHarder = false;
    boolean isMsBarcodes = true;

    JLabel infoLabel;
    FileWriter fw, fwd;

    public JFrame createInfo(JLabel l) {
        JFrame f = new JFrame();
        f.setAlwaysOnTop(true);
        f.setUndecorated(true);
        f.getContentPane().add(l);
        f.pack();
        f.setVisible(true);
        return f;
    }

    class QuitHandler extends Thread {
        public void run() {
            try {
                System.in.read();
                fw.close();
            } catch (Exception e) {
                e.printStackTrace();
            }
            System.exit(0);
        }
    }

    public ScreenHandler(String[] str) {
        for (String s : str) {
            if (s.equals("-nodecode")) isDecodeBarcode = false;
            else if (s.equals("-tryharder")) isTryHarder = true;
            else if (s.startsWith("-t")) cursorTime = Integer.parseInt(s.substring(2, s.length())) * 1000;
            else if (s.startsWith("-f")) filename = s.substring(2, s.length());
        }
        try {
            fw = new FileWriter(filename + ".csv");
            fwd = new FileWriter(filename + "_details.csv");
        } catch (Exception e) {
            e.printStackTrace();
        }
        System.out.println("decodeBarcode:" + isDecodeBarcode + " cursorTime:" + cursorTime + " isTryHarder:" + isTryHarder);
        //System.out.println("Move mouse cursor to upper left");
        infoLabel = new JLabel("Move mouse cursor to upper left    ");
        infoLabel.setFont(new Font("monospaced", Font.BOLD, 24));
        JFrame f = createInfo(infoLabel);
        try {
            Thread.sleep(cursorTime);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Point upperLeft = MouseInfo.getPointerInfo().getLocation();
        infoLabel.setText("Move mouse cursor to lower right");

        //System.out.println("Move mouse cursor to lower right");
        try {
            Thread.sleep(cursorTime);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Point lowerRight = MouseInfo.getPointerInfo().getLocation();

        x = upperLeft.x;//Integer.parseInt(str[0]);
        y = upperLeft.y;//Integer.parseInt(str[1]);
        w = lowerRight.x - upperLeft.x;//Integer.parseInt(str[2]);
        h = lowerRight.y - upperLeft.y;//Integer.parseInt(str[3]);
        r = new Rectangle(x, y, w, h);
        System.out.println("Barcode in " + r);
        SunToolkit toolkit = (SunToolkit) Toolkit.getDefaultToolkit();
    /*
        try
        {
            robotPeer = toolkit.createRobot(new Robot(), 
                                            GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice());
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
	*/
        try {
            nano = new Robot();
            Thread.sleep(100);
        } catch (Exception e) {
            e.printStackTrace();
        }

        oldBI = getScreen(new Point(x, y), new Point(x + w, y + h));
        newBI = oldBI;
        //oldArray = robotPeer.getRGBPixels(r);
        //newArray = oldArray;


        Thread t = new Thread(this);
        t.start();

        new QuitHandler().start();
    }

    public static void main(String[] args) {

        ScreenHandler sh = new ScreenHandler(args);
    }

    public void run() {

        boolean isSame;

        try {
            int test = 0;
            long start = System.nanoTime();
            long time2, time3, curTime;
            long statSecond = 0;
            int frames = 0, successful = 0, totalLatency = 0, latency = 0, totalFrames = 0, totalSuccess = 0, totalSec = 0;
            EAN8Reader barReader = new EAN8Reader();
            Hashtable<DecodeHintType, Object> hints = null;
            hints = new Hashtable<DecodeHintType, Object>(3);
            if (isTryHarder) hints.put(DecodeHintType.TRY_HARDER, Boolean.TRUE);
            else hints.put(DecodeHintType.TRY_HARDER, Boolean.FALSE);

            while (true) {
                //test++;
                //Thread.sleep(10);
                start = System.nanoTime();
                curTime = System.currentTimeMillis();
                StringBuffer sb = new StringBuffer(curTime + ",");
                //System.out.print(System.nanoTime() / 1000000 + " ");

                //if (!isRobotPeer)
                newBI = getScreen(new Point(x, y), new Point(x + w, y + h));

                //newArray = robotPeer.getRGBPixels(r);
                time2 = System.nanoTime();
                //System.out.print(time2);
                sb.append(" capt:" + (time2 - start) + "ns ");
                //isSame = areImagesEqual(newBI,oldBI);
                if (!isRobotPeer) {
                    isSame = Arrays.equals(((DataBufferInt) (newBI).getRaster().getDataBuffer()).getData(),
                            ((DataBufferInt) (oldBI).getRaster().getDataBuffer()).getData());
                } else {
                    isSame = Arrays.equals(newArray, oldArray);
                }
                time3 = System.nanoTime();
                sb.append("," + (time3 - time2) + ",");

                if (time3 / 1000000000 != statSecond) {
                    //Increment global counters
                    totalFrames += frames;
                    totalSuccess += successful;
                    totalSec++;
                    StringBuffer sbFile = new StringBuffer();
                    sbFile.append(System.currentTimeMillis() + "," + frames + "," + successful + ",");
                    if (successful > 0) sbFile.append((totalLatency / successful));
                    fw.write(sbFile.toString() + "\n");
                    //sbFile.append(","+totalFrames+","+totalSuccess);

                    //Set status label
                    if (successful == 0) successful = 1;
                    if (totalFrames == 0) totalFrames = 1;
                    if (frames == 0) frames = 1;

                    //infoLabel.setText(successful+"/"+frames+"  "+((successful*100)/frames)+"% "+ (totalLatency/successful)+"ms | "+
                    //	totalSuccess+"/"+totalFrames+"  "+((totalSuccess*100)/totalFrames)+"%  ");
                    infoLabel.setText("FPS:" + frames + "    CDL:" + (totalLatency / successful) + "ms" +
                            "    FRR:" + ((totalSuccess * 100) / totalFrames) + "%  ");

                    // Reset All counter
                    frames = 0;
                    successful = 0;
                    totalLatency = 0;
                    statSecond = time3 / 1000000000;
                }

                if (isSame) {
                    //System.out.print(" .");
                    //sb.append("same ");
                    try {
                        Thread.sleep(5);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                } else {
                    //sb.append("diff ");
                    frames++;
                    //ImageIO.write(newBI, "tif", new File( time2+".tif")  ) ;
                    if (isDecodeBarcode) {
                        String barcode, first4, last3;
                        try {
                            MonochromeBitmapSource source = new BufferedImageMonochromeBitmapSource(newBI);
                            //barReader = new EAN8Reader();
                            Result result = barReader.decode(source, hints);
                            ParsedResult parsedResult = ResultParser.parseResult(result);
                            barcode = parsedResult.getDisplayResult();
                            if (isMsBarcodes) {
                                first4 = barcode.substring(0, 4);
                                last3 = barcode.substring(4, 7);
                                if (first4.substring(1, 4).equals(last3)) barcode = first4;
                                else barcode = "ParityFailed";
                            } else {
                                if (barcode.substring(0, 3).equals(barcode.substring(3, 6)) &&
                                        barcode.substring(0, 1).equals(barcode.substring(6, 7))) {
                                    barcode = barcode.substring(0, 3);
                                } else barcode = "ParityFailed";
                            }
                        } catch (ReaderException e) {
                            barcode = "NoBarcode";
                        }
                        sb.append("" + ((System.nanoTime() - time3) / 1000000) + ",");
                        if (barcode.length() == 3 && !isMsBarcodes) {
                            latency = (int) (((curTime / 10) % 1000) - Integer.parseInt(barcode));
                            if (latency < 0) latency += 1000;
                            sb.append("" + latency + "");
                            successful++;
                            totalLatency += latency * 10;
                        } else if (barcode.length() == 4 && isMsBarcodes) {
                            curTime = System.currentTimeMillis();
                            latency = (int) (((curTime) % 10000) - Integer.parseInt(barcode));
                            if (latency < 0) latency += 10000;
                            sb.append("" + latency + "");
                            successful++;
                            totalLatency += latency;
                        } else {
                            sb.append("" + barcode + "");
                        }
                        //ImageIO.write(newBI, "png", new File( time2+"_"+barcode+".png")  ) ;
                        //sb.append(" fileio:"+((System.nanoTime() / 1000000) - start)+" ");

                        //System.out.print(" x");
                        fwd.write(sb + "\n");
                        //System.out.println(sb);
                    }
                }
                if (!isRobotPeer)
                    oldBI = newBI;
                else
                    oldArray = newArray;
                //System.out.println(" "+System.nanoTime() / 1000000 + " ");

            }//end of while()
            //System.out.println(System.nanoTime()-start+" nano");

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static BufferedImage getScreen(Point hg, Point bd) {
        return nano.createScreenCapture(new Rectangle(hg,
                new Dimension(
                        bd.x - hg.x,
                        bd.y - hg.y)));
    }
}
