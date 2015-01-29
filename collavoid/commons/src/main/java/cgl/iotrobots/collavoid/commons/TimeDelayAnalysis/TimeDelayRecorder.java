package cgl.iotrobots.collavoid.commons.TimeDelayAnalysis;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class TimeDelayRecorder {
    private Logger logger = LoggerFactory.getLogger(TimeDelayRecorder.class);
    private String path;
    private String fileName;
    private String component;
    private FileWriter fw = null;
    private boolean autoClose;

    public TimeDelayRecorder(String type, String parameter, String component) {
        if (type.equals(Constants.COMPUTATION_DELAY))
            path = Constants.basedir + type + "/";
        else
            path = Constants.basedir + type + "/" + parameter + "/";

        fileName = path + component;
        this.component = component;
    }

    public void open(boolean autoClose_) {
        this.autoClose = autoClose_;
        if (fw != null) {
            logger.warn("Already opened the file!!");
            return;
        }
        File dir = new File(path);
        if (!dir.exists()) {
            dir.mkdirs();
        }
        try {
            File file = new File(fileName);
            fw = new FileWriter(fileName);
            synchronized (file) {
                // write the title
                fw.write("robotID"+","+Constants.TIME_TAG + "," + component + "\n");
                fw.flush();
                fw.close();
            }
            if (!autoClose)
                fw = new FileWriter(fileName, true);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public synchronized void append(String robotName,long timeTag, long time) {
        String content = new String(robotName+","+timeTag + "," + time + "\n");
        try {
            if (autoClose) {
                fw = new FileWriter(fileName, true);
            }
            fw.write(content);
            fw.flush();
            if (autoClose)
                fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void close() {
        if (!autoClose) {
            try {
                fw.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }

}
