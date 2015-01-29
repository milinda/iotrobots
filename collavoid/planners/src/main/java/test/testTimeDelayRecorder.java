package test;

import cgl.iotrobots.collavoid.commons.TimeDelayAnalysis.Constants;
import cgl.iotrobots.collavoid.commons.TimeDelayAnalysis.TimeDelayRecorder;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

public class testTimeDelayRecorder {
    public static void main(String[] args) {
        TimeDelayRecorder timeDelayRecorder = new TimeDelayRecorder(Constants.COMPUTATION_DELAY,
                Constant_msg.KEY_VELOCITY_CMD, Constant_storm.Components.CONTROLLER_COMPONENT);
        TimeDelayRecorder nobuffer = new TimeDelayRecorder(Constants.COMPUTATION_DELAY,
                Constant_msg.KEY_VELOCITY_CMD, Constant_storm.Components.CONTROLLER_COMPONENT);
        timeDelayRecorder.open(false);
        long now = System.currentTimeMillis();
        for (int i = 0; i < 5000; i++) {
            timeDelayRecorder.append("robot0",System.currentTimeMillis(), System.currentTimeMillis() + 10);
        }
        timeDelayRecorder.close();
        System.out.println("average file operation delay" + (System.currentTimeMillis() - now) / 100.0 + " ms");

        nobuffer.open(false);
        now = System.currentTimeMillis();
        for (int i = 0; i < 5000; i++) {
            nobuffer.append("robot0",System.currentTimeMillis(), System.currentTimeMillis() + 10);
        }
        nobuffer.close();
        System.out.println("no buffer delay" + (System.currentTimeMillis() - now) / 100.0 + " ms");
    }
}
