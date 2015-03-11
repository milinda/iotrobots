package src.cgl.iotrobots.slam.core.gridfastslam;

import cgl.iotrobots.slam.core.gridfastsalm.MotionModel;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import junit.framework.TestCase;

public class MotionModelTest extends TestCase {
    public void testDrawFromMotionModel() throws Exception {
        DoubleOrientedPoint p = new DoubleOrientedPoint(0, 0, 0);
        DoubleOrientedPoint pOld = new DoubleOrientedPoint(.1, 0.1, 0.1);
        DoubleOrientedPoint pNew = new DoubleOrientedPoint(.1, .1, .0);

        MotionModel motionModel = new MotionModel();
        motionModel.srr = 0.01;
        motionModel.srt = 0.02;
        motionModel.str = 0.01;
        motionModel.stt = 0.02;
        DoubleOrientedPoint pCalc = motionModel.drawFromMotion(p, pNew, pOld);
        System.out.println(pCalc.getX() + " " + pCalc.getY() + " " + pCalc.getTheta());
    }
}
