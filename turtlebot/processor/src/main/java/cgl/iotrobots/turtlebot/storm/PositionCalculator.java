package cgl.iotrobots.turtlebot.storm;

import cgl.iotrobots.turtlebot.commons.Motion;

public class PositionCalculator {
//    public Motion calculatePosition(byte []depth_buf_) {
//        int height_ = 480;
//        int width_ = 640;
//        double SHIFT_SCALE = 0.125;
//        double  shift_offset_ = 1084.0;
//
//        int k = 0;
//        for (int v = 0; v < height_; ++v) {
//            for (int u = 0; u < width_; ++u, ++k) {
//                double d = SHIFT_SCALE * (shift_offset_ - depth_buf_[k]); // disparity
//                if (d <= 0.0)
//                    continue;
//
//                x = (i - w / 2) * (z + minDistance) * scaleFactor
//                y = (j - h / 2) * (z + minDistance) * scaleFactor
//                z = z
//                Where
//                        minDistance = -10
//                scaleFactor = .0021.
//                // Fill in XYZ
//                geometry_msgs::Point32 pt;
//                pt.z = fT / d;
//                pt.x = ((u - depth_model_.cx()) / depth_model_.fx()) * pt.z;
//                pt.y = ((v - depth_model_.cy()) / depth_model_.fy()) * pt.z;
//                cloud_.points.push_back(pt);
//
//                // Fill in RGB from corresponding pixel in rectified RGB image
//                Eigen::Vector4d uvd1;
//                uvd1 << u, v, d, 1;
//                Eigen::Vector3d uvw;
//                uvw = depth_to_rgb_ * uvd1;
//                int u_rgb = uvw[0] / uvw[2] + 0.5;
//                int v_rgb = uvw[1] / uvw[2] + 0.5;
//
//                int32_t rgb_packed = 0;
//                if (u_rgb >= 0 && u_rgb < width_ && v_rgb >= 0 && v_rgb < height_) {
//                    cv::Vec3b rgb = rgb_rect_.at < cv::Vec3b > (v_rgb, u_rgb);
//                    rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
//                }
//                cloud_.channels[0].values.push_back( * (float*)( & rgb_packed));
//            }
//        }
//    }
}
