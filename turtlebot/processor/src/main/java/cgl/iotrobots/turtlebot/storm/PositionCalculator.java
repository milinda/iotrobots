package cgl.iotrobots.turtlebot.storm;

import cgl.iotrobots.turtlebot.commons.Motion;
import cgl.iotrobots.turtlebot.commons.Velocity;
import com.jcraft.jzlib.Inflater;
import com.jcraft.jzlib.JZlib;
import com.jcraft.jzlib.ZStream;

public class PositionCalculator {
    private double t_gamma[];
    public PositionCalculator() {
        t_gamma = new double[2048];
        for (int p = 0; p < 2048; p++) {
            t_gamma[p] = 0.1236 * Math.tan(p / 2842.5 + 1.1863);
        }
    }

    public Motion calculatePosition(byte[] data) {
        byte[] depth_buf_ = unCompress(data);
        double max_z_ = 1.2;
        double min_y_ = .1;
        double max_y_ = .5;
        double max_x_ = .2;
        double min_x_ = -.2;
        double goal_z_ = .8;

        double z_scale_ = 1, x_scale_ = 20;

        int height_ = 480;
        int width_ = 640;
        double minDistance = -10;
        double scaleFactor = .0021;
        double x, y, z;
        double totX = 0, totY = 0, totZ = 0;
        int k = 0, n = 0;

        double cx = 320.0; // center of projection
        double cy = 240.0; // center of projection
        double fx = 600.0; // focal length in pixels
        double fy = 600.0; // focal length in pixels

        for (int v = 0; v < height_; ++v) {
            for (int u = 0; u < width_; ++u) {
                int lo = depth_buf_[k] & 0xFF;
                int hi = depth_buf_[k + 1] & 0xFF;
                int disp = hi << 8 | lo;

                double d = t_gamma[disp];
                if (d <= 0.0)
                    continue;
                // Fill in XYZ
//                z = d;
//                x = (u - width_ / 2) * (z + minDistance) * scaleFactor;
//                y = (v - height_ / 2) * (z + minDistance) * scaleFactor;
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
                k += 2;
            }
        }

        if (n > 1000) {
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
            if (totX * x_scale_ >= .1) {
                return new Motion(new Velocity((totZ - goal_z_) * z_scale_, 0, 0), new Velocity(0, 0, totX * x_scale_));
            } else {
                return new Motion(new Velocity((totZ - goal_z_) * z_scale_, 0, 0), new Velocity(0, 0, 0));
            }

        } else {
            // System.out.println("No valid points detected, stopping the robot");
            return new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, 0));
        }
    }

    public static byte[] unCompress(byte []data) {
        int err;
        byte[] restored = new byte[614400];
        Inflater inflater = new Inflater();
        inflater.setInput(data);

        while (true) {
            inflater.setOutput(restored);
            err = inflater.inflate(JZlib.Z_NO_FLUSH);
            if (err == JZlib.Z_STREAM_END) break;
            CHECK_ERR(inflater, err, "inflate large");
        }

        err = inflater.end();
        CHECK_ERR(inflater, err, "inflateEnd");
        return restored;
    }

    static void CHECK_ERR(ZStream z, int err, String msg) {
        if (err != JZlib.Z_OK) {
            if (z.msg != null) System.out.print(z.msg + " ");
            System.out.println(msg + " error: " + err);

            System.exit(1);
        }
    }
}
