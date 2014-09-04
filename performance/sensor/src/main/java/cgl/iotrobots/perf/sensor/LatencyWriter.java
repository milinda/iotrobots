package cgl.iotrobots.perf.sensor;

import java.io.*;
import java.util.List;

public class LatencyWriter {
    private PrintWriter writer;

    public LatencyWriter(String fileName) {
        try {
            writer = new PrintWriter(new BufferedWriter(new FileWriter(fileName, true)));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void write(String test, List<Integer> lat) {
        StringBuilder builder = new StringBuilder();
        builder.append(test);
        for (Integer l : lat) {
            builder.append(" ").append(lat.toString());
        }
        writer.println(builder.toString());
    }

    public void close() {
        writer.close();
    }
}
