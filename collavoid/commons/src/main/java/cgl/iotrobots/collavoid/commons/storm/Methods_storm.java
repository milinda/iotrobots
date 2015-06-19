package cgl.iotrobots.collavoid.commons.storm;

import backtype.storm.Config;
import cgl.iotrobots.collavoid.commons.planners.*;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import com.esotericsoftware.kryo.Kryo;

public class Methods_storm {

    public static void addSerializers(Config config) {
        config.registerSerialization(Odometry_.class);
        config.registerSerialization(Header_.class);
        config.registerSerialization(Pose_.class);
        config.registerSerialization(Twist_.class);
        config.registerSerialization(PoseShareMsg_.class);
        config.registerSerialization(PoseArray_.class);
        config.registerSerialization(Vector3d_.class);
        config.registerSerialization(Vector4d_.class);
        config.registerSerialization(Vector2.class);
        config.registerSerialization(BasicConfig_.class);
        config.registerSerialization(PointCloud2_.class);
        config.registerSerialization(LaserScan_.class);
        config.registerSerialization(PoseStamped_.class);
        config.registerSerialization(Kryo.class);

        config.registerSerialization(Position.class);
        config.registerSerialization(Agent.class);
        config.registerSerialization(Neighbor.class);
        config.registerSerialization(VO.class);
        config.registerSerialization(Line.class);
        config.registerSerialization(LinePair.class);
        config.registerSerialization(Obstacle.class);
    }

}
