package test;

import cgl.sensorstream.core.Utils;
import cgl.sensorstream.core.config.Configuration;

import java.io.IOException;
import java.net.URL;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class testReadConfig {
    public static void main(String[] args) throws IOException {
        Map<String, String> conf = new HashMap<>();
        conf = Utils.readStreamConfig();
//        for (URL root : Collections.list(Thread.currentThread().getContextClassLoader().getResources(""))) {
//            System.out.println(root);
//        }
//        System.out.println(System.getProperty("stream.conf.file"));
        System.out.println(conf.size());
        Object o = conf.get("zk.root");
        System.out.println(o.toString());
    }


}
