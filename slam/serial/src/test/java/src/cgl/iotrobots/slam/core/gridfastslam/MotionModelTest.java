package src.cgl.iotrobots.slam.core.gridfastslam;

import cgl.iotrobots.slam.core.gridfastsalm.MotionModel;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.core.utils.Stat;
import junit.framework.TestCase;
import org.apache.commons.math3.distribution.NormalDistribution;

public class MotionModelTest extends TestCase {
    public void testDrawFromMotionModel() throws Exception {
        DoubleOrientedPoint p;
        DoubleOrientedPoint pOld;
        DoubleOrientedPoint pNew;
        p = new DoubleOrientedPoint(0, 0, 0);
        pOld = new DoubleOrientedPoint(1, 0, 0);
        for (int i = 0; i < 1000; i++) {
            pNew = new DoubleOrientedPoint(pOld.getX() + 1, 0, .0);

            MotionModel motionModel = new MotionModel();
            motionModel.srr = -0.001;
            motionModel.srt = 0.002;
            motionModel.str = -0.001;
            motionModel.stt = -0.002;
            p = motionModel.drawFromMotion(p, pNew, pOld);
            System.out.println(p.getX() + " " + p.getY() + " " + p.getTheta());
            pOld = pNew;
        }
    }

    public void testSampleGausian() {
        double sum = 0;
        int n = 10000;
        for (int i = 0; i < n; i++) {
            double s = Stat.sampleGaussian(.005, 0);
            sum += s;
            System.out.println(s);
        }
        System.out.println("Average: " + sum / n);

        sum = 0;
        NormalDistribution normalDistribution = new NormalDistribution(.005, .001);
        for (int i = 0; i < n; i++) {
            double s = normalDistribution.sample();
            sum += s;
            System.out.println(s);
        }
        System.out.println("Average: " + sum / n);
    }
}
