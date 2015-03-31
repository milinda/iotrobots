package cgl.iotrobots.slam.streaming.msgs;

import com.sun.corba.se.impl.encoding.OSFCodeSetRegistry;

import java.util.HashMap;
import java.util.Map;

public class Trace {
    // time spent in the dispatcher, by previous tuple
    private long pd;
    // total time spent in the scan matcher
    private long sm;
    // processing time spent on each of the scan matchers
    private Map<Integer, Long> smp = new HashMap<Integer, Long>();
    // resampling processing time
    private long rsp;
    //actual resampling processing time
    private long actualRsp;
    // scanmatcher, receiving assingment to being ready to receive time
    private long smar;
    // scanmatche time for post processing
    private long smaPP;

    public Trace() {
    }

    public long getSm() {
        return sm;
    }

    public void setSm(long sm) {
        this.sm = sm;
    }

    public long getSmar() {
        return smar;
    }

    public void setSmar(long smar) {
        this.smar = smar;
    }

    public long getPd() {
        return pd;
    }


    public long getRsp() {
        return rsp;
    }

    public void setPd(long pd) {
        this.pd = pd;
    }

    public void setRsp(long rsp) {
        this.rsp = rsp;
    }

    public long getActualRsp() {
        return actualRsp;
    }

    public void setActualRsp(long actualRsp) {
        this.actualRsp = actualRsp;
    }

    public Map<Integer, Long> getSmp() {
        return smp;
    }

    public void setSmp(Map<Integer, Long> smp) {
        this.smp = smp;
    }

    public long getSmaPP() {
        return smaPP;
    }

    public void setSmaPP(long smaPP) {
        this.smaPP = smaPP;
    }

    public String serialize() {
        String s = "'";
        for (Map.Entry<Integer, Long> e : smp.entrySet()) {
            s += e.getValue() + " ";
        }
        s += "'";
        return pd + ", " + sm + ", " + s + " ," + rsp + " ," + actualRsp + " ," + smar + ", " + smaPP;
    }
}
