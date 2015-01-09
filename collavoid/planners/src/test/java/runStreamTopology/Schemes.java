package runStreamTopology;

import backtype.storm.spout.Scheme;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Schemes {
    public static Map<String, Scheme> schemeMap = new HashMap<String, Scheme>();

    public Schemes() {
        schemeMap.put(Constant_msg.KEY_ODOMETRY, new OdometryScheme());
        schemeMap.put(Constant_msg.KEY_SCAN, new ScanScheme());
        schemeMap.put(Constant_msg.KEY_PARTICLE_CLOUD, new ParticleScheme());
        schemeMap.put(Constant_msg.KEY_POSE_SHARE, new PoseShareScheme());
        schemeMap.put(Constant_msg.KEY_BASE_CONFIG, new BaseConfigScheme());
    }

    public static class OdometryScheme implements Scheme {
        @Override
        public List<Object> deserialize(byte[] bytes) {
            Odometry_ odometry_ = (Odometry_) Methods_RMQ.deserialize(bytes, Odometry_.class);
            return new Values(
                    odometry_.getHeader().getStamp(),
                    odometry_.getId(),
                    odometry_
            );
        }

        @Override
        public Fields getOutputFields() {
            return new Fields(
                    Constant_storm.FIELDS.TIME_FIELD,
                    Constant_storm.FIELDS.SENSOR_ID_FIELD,
                    Constant_storm.FIELDS.ODOMETRY_FIELD);
        }
    }

    public static class ScanScheme implements Scheme {
        @Override
        public List<Object> deserialize(byte[] bytes) {
            PointCloud2_ data = (PointCloud2_) Methods_RMQ.deserialize(bytes, PointCloud2_.class);
            return new Values(
                    data.getHeader().getStamp(),
                    data.getId(),
                    data);
        }

        @Override
        public Fields getOutputFields() {
            return new Fields(
                    Constant_storm.FIELDS.TIME_FIELD,
                    Constant_storm.FIELDS.SENSOR_ID_FIELD,
                    Constant_storm.FIELDS.SCAN_FIELD);
        }
    }

    public static class ParticleScheme implements Scheme {

        @Override
        public List<Object> deserialize(byte[] bytes) {
            PoseArray_ data = (PoseArray_) Methods_RMQ.deserialize(bytes, PoseArray_.class);
            return new Values(
                    data.getHeader().getStamp(),
                    data.getId(),
                    data);
        }

        @Override
        public Fields getOutputFields() {
            return new Fields(
                    Constant_storm.FIELDS.TIME_FIELD,
                    Constant_storm.FIELDS.SENSOR_ID_FIELD,
                    Constant_storm.FIELDS.POSE_ARRAY_FIELD);
        }

    }

    public static class PoseShareScheme implements Scheme {


        @Override
        public List<Object> deserialize(byte[] bytes) {
            PoseShareMsg_ data = (PoseShareMsg_) Methods_RMQ.deserialize(bytes, PoseShareMsg_.class);
            return new Values(
                    data.getHeader().getStamp(),
                    data);
        }

        @Override
        public Fields getOutputFields() {
            return new Fields(
                    Constant_storm.FIELDS.TIME_FIELD,
                    Constant_storm.FIELDS.POSE_SHARE_FIELD);
        }
    }

    public static class BaseConfigScheme implements Scheme {

        @Override
        public List<Object> deserialize(byte[] bytes) {
            BaseConfig_ data = (BaseConfig_) Methods_RMQ.deserialize(bytes, BaseConfig_.class);
            return new Values(
                    data.getTime(),
                    data.getId(),
                    data);
        }

        @Override
        public Fields getOutputFields() {
            return new Fields(
                    Constant_storm.FIELDS.TIME_FIELD,
                    Constant_storm.FIELDS.SENSOR_ID_FIELD,
                    Constant_storm.FIELDS.BASE_CONFIG_FIELD);
        }

    }
}
