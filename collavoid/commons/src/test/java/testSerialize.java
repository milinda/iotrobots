import cgl.iotrobots.collavoid.commons.Header_;
import cgl.iotrobots.collavoid.commons.Odometry_;

import java.io.IOException;

/**
 * Created by hjh on 12/18/14.
 */
public class testSerialize {
    public static void main(String[] args) {
        Odometry_ odom = new Odometry_();
        try {
            odom.toJSON();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
}
