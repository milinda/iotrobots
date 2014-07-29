package cgl.iotrobots.turtlebot.commons;

import org.xerial.snappy.Snappy;

import java.nio.ByteBuffer;
import java.io.IOException;

public class Compressor {
    private byte inverted[] = new byte[1024];

    public Compressor() {
        for (int i = 0; i < 1024; i++) {
            this.inverted[i] = (byte) (90300 / (1000 * 0.1236 * Math.tan(i / 2842 + 1.1863)) - 21.575);
        }
    }

    public byte[] comprFrame(ByteBuffer frame) throws IOException {
        byte[] data = new byte[307200];
        int p = 0;
        for (int i = 0; i < 614400; i += 2) {
            int lo = frame.get(i) & 0xFF;
            int hi = frame.get(i + 1) & 0xFF;
            int disp = hi << 8 | lo;
            if (disp > 60 && disp < 1012) data[p] = this.inverted[disp];
            else data[p] = 0;
            p++;
        }
        return Snappy.compress(data);
    }

    public int[] unCompr(byte[] data) throws IOException {
        byte[] invertedD;
        invertedD = Snappy.uncompress(data);

        int[] dist = new int[307200];
        for (int i = 0; i < 307200; i++) {
            if (invertedD[i] == 0) {
                dist[i] = 0;
                continue;
            }
            dist[i] = (invertedD[i] & 0xFF);
            dist[i] = (int) (90300 / (dist[i] + 21.575)); //distance given in mm
        }
        return dist;
    }
}
