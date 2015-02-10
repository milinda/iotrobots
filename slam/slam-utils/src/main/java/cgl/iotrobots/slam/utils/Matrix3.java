package cgl.iotrobots.slam.utils;

import cgl.iotrobots.slam.utils.Euler;
import geometry_msgs.Quaternion;

public class Matrix3 {
    double[][] m_el = new double[3][3];

    public Matrix3(Quaternion q) {
        setRotation(q);
    }

    public void setRotation(Quaternion q) {
        double d = 2;
        double s = 2.0 / d;
        double xs = q.getX() * s, ys = q.getY() * s, zs = q.getZ() * s;
        double wx = q.getW() * xs, wy = q.getW() * ys, wz = q.getW() * zs;
        double xx = q.getX() * xs, xy = q.getX() * ys, xz = q.getX() * zs;
        double yy = q.getY() * ys, yz = q.getY() * zs, zz = q.getZ() * zs;

        setValue((1.0) - (yy + zz), xy - wz, xz + wy,
                xy + wz, (1.0) - (xx + zz), yz - wx,
                xz - wy, yz + wx, (1.0) - (xx + yy));
    }

    public void setValue(double xx, double xy, double xz,
                  double yx, double yy, double yz,
                  double zx, double zy, double zz) {
        m_el[0][0] = xx;
        m_el[0][1] = xy;
        m_el[0][2] = xz;
        m_el[1][0] = yx;
        m_el[1][1] = yy;
        m_el[1][2] = yz;
        m_el[2][0] = zx;
        m_el[2][1] = zy;
        m_el[2][2] = zz;
    }

    public Euler getEulerYPR() {
        Euler euler_out = new Euler();
        Euler euler_out2 = new Euler(); //second solution
        //get the pointer to the raw data

        // Check that pitch is not at a singularity
        // Check that pitch is not at a singularity
        if (Math.abs(m_el[2][0]) >= 1) {
            euler_out.yaw = 0;
            euler_out2.yaw = 0;

            // From difference of angles formula
            if (m_el[2][0] < 0)  //gimbal locked down
            {
                double delta = Math.atan2(m_el[0][1], m_el[0][2]);
                euler_out.pitch = Math.PI / 2.0;
                euler_out2.pitch = Math.PI / 2.0;
                euler_out.roll = delta;
                euler_out2.roll = delta;
            } else {
                double delta = Math.atan2(-m_el[0][1], -m_el[0][2]);
                euler_out.pitch = -Math.PI / 2.0;
                euler_out2.pitch = -Math.PI / 2.0;
                euler_out.roll = delta;
                euler_out2.roll = delta;
            }
        } else {
            euler_out.pitch = -Math.asin(m_el[2][0]);
            euler_out2.pitch = Math.PI - euler_out.pitch;

            euler_out.roll = Math.atan2(m_el[2][1] / Math.cos(euler_out.pitch),
                    m_el[2][2] / Math.cos(euler_out.pitch));
            euler_out2.roll = Math.atan2(m_el[2][1] / Math.cos(euler_out2.pitch),
                    m_el[2][2] / Math.cos(euler_out2.pitch));

            euler_out.yaw = Math.atan2(m_el[1][0] / Math.cos(euler_out.pitch),
                    m_el[0][0] / Math.cos(euler_out.pitch));
            euler_out2.yaw = Math.atan2(m_el[1][0] / Math.cos(euler_out2.pitch),
                    m_el[0][0] / Math.cos(euler_out2.pitch));
        }

        Euler e = new Euler();

        e.yaw = euler_out.yaw;
        e.pitch = euler_out.pitch;
        e.roll = euler_out.roll;

        return e;
    }
}
