package cgl.iotrobots.turtlebot.storm;

import cgl.iotrobots.turtlebot.commons.Motion;
import cgl.iotrobots.turtlebot.commons.Velocity;

public class PositionCalculator {
    final double max_z_ = 1.0;
    final double min_y_ = .1;
    final double max_y_ = .5;
    final double max_x_ = .2;
    final double min_x_ = -.2;
    final double goal_z_ = .7;

    public Motion calculatePosition(int[] dist) {
        double z_scale_ = 1, x_scale_ = 5.0;
        double x, y, z;
        double totX = 0, totY = 0, totZ = 0;
        int n = 0, u = 0, v = 0;

        double cx = 320.0; // center of projection
        double cy = 240.0; // center of projection
        double fx = 600.0; // focal length in pixels
        double fy = 600.0; // focal length in pixels

        for (int i = 0; i < 307200; i++) {
            v = (int) Math.floor((double) i / 640);
            u = i - 640 * v;
            double d = (double) (dist[i]) / 1000; // distance converted to m
            if (d <= 0.0) {
                continue;
            }
            // Fill in XYZ
            x = (u - cx) * d / fx;
            y = (v - cy) * d / fy;
            z = d;
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

            if (Math.abs(totZ - max_z_) < 0.01) {
                return new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, 0));
            }
            if (Math.abs(totX * x_scale_) >= .1) {
                return new Motion(new Velocity((totZ - goal_z_) * z_scale_, 0, 0), new Velocity(0, 0, -1 * totX * x_scale_));
            } else {
                return new Motion(new Velocity((totZ - goal_z_) * z_scale_, 0, 0), new Velocity(0, 0, 0));
            }
        } else {
            // System.out.println("No valid points detected, stopping the robot");
            return new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, 0));
        }
    }

}
