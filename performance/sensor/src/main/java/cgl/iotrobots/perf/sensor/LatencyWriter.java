package cgl.iotrobots.perf.sensor;

import java.io.*;

public class LatencyWriter {
    private PrintWriter writer;

    public LatencyWriter(String fileName) {
        try {
            writer = new PrintWriter(new BufferedWriter(new FileWriter("/home/supun/dev/projects/LatencyTest.txt", true)));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void write(int lat) {
        writer.println(lat);

    }

    public void close() {
        writer.close();
    }
}
