package cgl.iotrobots.st.storm;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Starts a decoding process and do the decoding
 */
public class Decoder implements Serializable {
    private Logger LOG = LoggerFactory.getLogger(Decoder.class);

    private BlockingQueue<DecoderMessage> outputQueue;

    private BlockingQueue<Long> time = new LinkedBlockingQueue<Long>();

    /**
     * The size of the image
     */
    private static final int frameSize = 360 * 640 * 3;
    /**
     * The process we create
     */
    private Process proc;

    public Decoder(BlockingQueue<DecoderMessage> outputQueue) {
        this.outputQueue = outputQueue;
    }

    public void start() {
        try {
            Runtime rt = Runtime.getRuntime();
            proc = rt.exec("nice -n 15 avconv -i - -probesize 2048 -flags low_delay -f rawvideo -pix_fmt rgb24 -");

            ErrorStreamGobbler errorGobbler = new ErrorStreamGobbler(proc.getErrorStream(), "ERROR");
            ImageOutGobbler outputGobbler = new ImageOutGobbler(proc.getInputStream(), "OUTPUT");

            errorGobbler.start();
            outputGobbler.start();
        } catch (Throwable t) {
            LOG.error("Failed to create the avconv process");
        }
    }

    public void write(DecoderMessage message) {
        if (proc != null) {
            OutputStream stream = proc.getOutputStream();
            try {
                stream.write(message.getMessage());
            } catch (IOException e) {
                LOG.error("Failed to write the frame to decoder", e);
            }
        } else {
            throw new IllegalStateException("The process should be created before calling the write");
        }
    }

    public void close() {
        if (proc != null) {
            proc.destroy();
        }
    }

    class ImageOutGobbler extends Thread {
        InputStream is;
        String type;

        ImageOutGobbler(InputStream is, String type) {
            this.is = is;
            this.type = type;
        }

        public void run() {

            while (true) {
                try {
                    DataInputStream isr = new DataInputStream(is);
                    byte output[] = new byte[frameSize];
                    isr.readFully(output);

                    DecoderMessage message = new DecoderMessage(output, Long.toString(System.currentTimeMillis()));
                    outputQueue.put(message);
                } catch (IOException e) {
                    LOG.error("Error reading output stream from the decoder.", e);
                } catch (InterruptedException ignored) {
                }
            }
        }
    }

    class ErrorStreamGobbler extends Thread {
        InputStream is;
        String type;

        ErrorStreamGobbler(InputStream is, String type) {
            this.is = is;
            this.type = type;
        }

        public void run() {
            while (true) {
                try {
                    InputStreamReader isr = new InputStreamReader(is);
                    BufferedReader br = new BufferedReader(isr);
                    String line;
                    while ((line = br.readLine()) != null)
                        LOG.error(type + " > " + line);
                } catch (IOException e) {
                    LOG.error("Failed to read error output from decoder", e);
                }
            }
        }
    }
}

