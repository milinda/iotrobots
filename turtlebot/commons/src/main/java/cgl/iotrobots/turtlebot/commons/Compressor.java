import com.jcraft.jzlib.*;
import java.nio.ByteBuffer;

public class Compressor {

    public Compressor() {
	byte inverted[] = new byte[1024];
	for(int i = 0; i < 1024; i++) {
	    inverted[i] = (byte) (90300 / (1000 * 0.1236 * Math.tan(i / 2842 + 1.1863)) - 21.575);
	}
    }

    public byte[] comprFrame(ByteBuffer frame) {
	byte[] data = new byte[307200];
	int p = 0;
	for(int i = 0; i < 614400; i+=2) {
	    int lo = frame.get(i) & 0xFF;
	    int hi = frame.get(i+1) & 0xFF;
	    int disp = hi << 8 | lo;
	    if (disp > 60 && disp < 1012) data[p] = inverted[disp];
	    else data[p] = 0;
	    p++;
	}

	int err;
	byte[] compr = new byte[65000];

	Deflater deflater = null;
	try {
	    deflater = new Deflater(JZlib.Z_BEST_SPEED);
	} catch (GZIPException e) {
	    System.exit(0);
	}
	
	deflater.setInput(data);
	deflater.setOutput(compr);
	
	err = deflater.deflate(JZlib.Z_NO_FLUSH);
	CHECK_ERR(deflater, err, "deflate");
	if (deflater.avail_in != 0) {
	    System.out.println("deflate not greedy");
	    System.exit(1);
	}
	
	err = deflater.deflate(JZlib.Z_FINISH);
	if (err != JZlib.Z_STREAM_END) {
	    System.out.println("deflate should report Z_STREAM_END");
	    System.exit(1);
	}
	err = deflater.end();
	CHECK_ERR(deflater, err, "deflateEnd");

	return compr;
    }

    public int[] unCompr(byte[] data) {
	int err;
        byte[] inverted = new byte[307200];
        Inflater inflater = new Inflater();
        inflater.setInput(data);
	
        while (true) {
            inflater.setOutput(inverted);
            err = inflater.inflate(JZlib.Z_NO_FLUSH);
            if (err == JZlib.Z_STREAM_END) break;
            CHECK_ERR(inflater, err, "inflate large");
        }

        err = inflater.end();
        CHECK_ERR(inflater, err, "inflateEnd");

        int[] dist = new int[307200];
        for(int i=0; i<307200; i++) {
            if(inverted[i]==0) {
                dist[i] = 0;
                continue;
            }
            dist[i] = (inverted[i] & 0xFF);
            dist[i] = (int)(90300 / (dist[i] + 21.575)); //distance given in mm
        }
        return dist;
    }

    private void CHECK_ERR(ZStream z, int err, String msg) {
        if (err != JZlib.Z_OK) {
            if (z.msg != null) System.out.print(z.msg + " ");
            System.out.println(msg + " error: " + err);
            System.exit(1);
        }
    }
}