package cgl.iotrobots.slam.utils;

import nav_msgs.Odometry;
import org.apache.commons.lang3.tuple.Pair;
import sensor_msgs.LaserScan;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.BlockingQueue;

/**
 * This is a simple message filter to drop messages if they are not close enough
 */
public class MessageFilter {
    // the minimum difference of time
    private long difference;

    private BlockingQueue<Pair<Odometry, LaserScan>> queue;

    private List<Odometry> odometries = new ArrayList<Odometry>(256);

    private List<LaserScan> laserScans = new ArrayList<LaserScan>(256);

    public MessageFilter(long difference, BlockingQueue<Pair<Odometry, LaserScan>> queue) {
        this.difference = difference;
        this.queue = queue;
    }

    public synchronized void addLaserScan(LaserScan laserScan) throws InterruptedException {
        // check weather we have a odometry within the time
        Odometry match = null;
        Iterator<Odometry> iterator = odometries.iterator();
        long abs = -1;
        while (iterator.hasNext()) {
            Odometry o = iterator.next();
            abs = Math.abs(o.getHeader().getStamp().subtract(laserScan.getHeader().getStamp()).totalNsecs());
            if (abs < difference * 1000000) {
                if (match != null) {
                    if (match.getHeader().getStamp().subtract(laserScan.getHeader().getStamp()).totalNsecs() > 0) {
                        match = o;
                    }
                } else {
                    match = o;
                }
            } else {
                iterator.remove();
            }
        }

        if (match != null) {
            System.out.println("Match found: difference " + abs);
            queue.put(Pair.of(match, laserScan));
        } else {
            laserScans.add(laserScan);
        }
    }

    public synchronized void addOdometry(Odometry odom) throws InterruptedException {
        // check weather we have a odometry within the time
        LaserScan match = null;
        Iterator<LaserScan> iterator = laserScans.iterator();
        long abs = -1;
        while (iterator.hasNext()) {
            LaserScan o = iterator.next();
            abs = Math.abs(o.getHeader().getStamp().subtract(odom.getHeader().getStamp()).totalNsecs());

            if (abs < difference * 1000000) {
                if (match != null) {
                    if (match.getHeader().getStamp().subtract(odom.getHeader().getStamp()).totalNsecs() > 0) {
                        match = o;
                    }
                } else {
                    match = o;
                }
            } else {
                iterator.remove();
            }
        }

        if (match != null) {
            System.out.println("Match found: difference " + abs);
            queue.put(Pair.of(odom, match));
        } else {
            odometries.add(odom);
        }
    }
}
