package cgl.iotrobots.collavoid.commons.storm;

import backtype.storm.Constants;
import backtype.storm.tuple.Tuple;

/**
 * Created by hjh on 12/23/14.
 */
public class Methods_storm {
    public static boolean isTickTuple(Tuple tuple) {
        return tuple.getSourceComponent().equals(Constants.SYSTEM_COMPONENT_ID) && tuple.getSourceStreamId().equals(
                Constants.SYSTEM_TICK_STREAM_ID);
    }

}
