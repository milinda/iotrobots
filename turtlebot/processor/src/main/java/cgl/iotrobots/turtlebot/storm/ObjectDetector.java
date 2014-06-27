package cgl.iotrobots.turtlebot.storm;

import com.jcraft.jzlib.Inflater;
import com.jcraft.jzlib.JZlib;
import com.jcraft.jzlib.ZStream;

public class ObjectDetector {
    double t_gamma[] = new double[1024];

    public ObjectDetector() {
        for (int p = 0; p < 1024; p++) {
            t_gamma[p] = 100 * 0.1236 * Math.tan(p / 2842.5 + 1.1863);
        }
    }

    public boolean detect(byte[] data) {
        byte[] restored = new byte[614400];

        Inflater inflater = new Inflater();

        inflater.setInput(data);
        int err;

        while (true) {
            inflater.setOutput(restored);
            err = inflater.inflate(JZlib.Z_NO_FLUSH);
            if (err == JZlib.Z_STREAM_END) break;
            CHECK_ERR(inflater, err, "inflate large");
        }

        err = inflater.end();
        CHECK_ERR(inflater, err, "inflateEnd");

        for (int i = 0; i < 614400; i += 2) {
            int lo = restored[i] & 0xFF;
            int hi = restored[i + 1] & 0xFF;
            int disp = hi << 8 | lo;
            double dist = t_gamma[disp];
            if (dist >= 40 && dist < 75) {
                return true;
            }
        }
        return false;
    }

    static void CHECK_ERR(ZStream z, int err, String msg) {
        if(err!= JZlib.Z_OK){
            if(z.msg!=null) System.out.print(z.msg+" ");
            System.out.println(msg+" error: "+err);

            System.exit(1);
        }
    }
}
