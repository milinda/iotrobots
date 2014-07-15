package cgl.iotrobots.turtlebot.storm;

import cgl.iotrobots.turtlebot.commons.Motion;
import cgl.iotrobots.turtlebot.commons.Velocity;
import cgl.iotrobots.turtlebot.commons.Compressor;

public class PositionCalculator {

    public Motion calculatePosition(byte[] data) {
        Compressor compressor = new Compressor();
        int[] dist = compressor.unCompr(data);
        double max_z_ = 1.0;
        double min_y_ = .1;
        double max_y_ = .5;
        double max_x_ = .2;
        double min_x_ = -.2;
        double goal_z_ = .7;

        double z_scale_ = 1, x_scale_ = 5.0;

        double x, y, z;
        double totX = 0, totY = 0, totZ = 0;
        int n = 0, u = 0, v = 0;

        double cx = 320.0; // center of projection
        double cy = 240.0; // center of projection
        double fx = 600.0; // focal length in pixels
        double fy = 600.0; // focal length in pixels

        for(int i=0; i<307200; i++) {
            v = (int) Math.floor((double) i / 640);
            u = i - 640 * v;
            double d = (double) (dist[i]) / 1000; // distance converted to m
            if (d <= 0.0)
                continue;
            // Fill in XYZ
//          z = d;
//          x = (u - width_ / 2) * (z + minDistance) * scaleFactor;
//          y = (v - height_ / 2) * (z + minDistance) * scaleFactor;
            x = (u - cx) * d / fx;
            y = (v - cy) * d / fy;
            z = d;
            //System.out.format("x %f, y %f, z %f\n", x, y, z);

            if (y > min_y_ && y < max_y_ && x < max_x_ && x > min_x_ && z < max_z_) {
                //Add the point to the totals
                totX += x;
                totY += y;
                totZ += z;
                n++;
            }
        }

        if (n > 4000) {
            totX /= n;
            totY /= n;
            totZ /= n;
            System.out.format("x %f, y %f, z %f\n", totX, totY, totZ);
            if (Math.abs(totZ  -max_z_) < 0.01) {
                //System.out.println("No valid points detected, stopping the robot");
                return new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, 0));
            }

            // System.out.format("Centroid at %f %f %f with %d points", totX, totY, totZ, n);
            System.out.println();
            if (Math.abs(totX * x_scale_) >= .1) {
                return new Motion(new Velocity((totZ - goal_z_) * z_scale_, 0, 0), new Velocity(0, 0, -1* totX * x_scale_));
            } else {
                return new Motion(new Velocity((totZ - goal_z_) * z_scale_, 0, 0), new Velocity(0, 0, 0));
            }

        } else {
            // System.out.println("No valid points detected, stopping the robot");
            return new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, 0));
        }
    }

}
