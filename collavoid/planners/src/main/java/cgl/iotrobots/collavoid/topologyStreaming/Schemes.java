package cgl.iotrobots.collavoid.topologyStreaming;

import backtype.storm.spout.Scheme;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import com.esotericsoftware.kryo.Kryo;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Schemes {
    public static Map<String, Scheme> schemeMap = new HashMap<String, Scheme>();

    public Schemes() {
        schemeMap.put(Constant_msg.KEY_ODOMETRY, new OdometryScheme());
        schemeMap.put(Constant_msg.KEY_SCAN, new ScanScheme());
        schemeMap.put(Constant_msg.KEY_POSE_ARRAY, new ParticleScheme());
//        schemeMap.put(Constant_msg.KEY_POSE_SHARE, new PoseShareScheme());
        schemeMap.put(Constant_msg.KEY_BASE_CONFIG, new BaseConfigScheme());
    }

    public static class OdometryScheme implements Scheme {
        private Kryo odometrySchemeKryo;
        private boolean initedOdomkryo = false;
        @Override
        public List<Object> deserialize(byte[] bytes) {
            if (!initedOdomkryo) {
                odometrySchemeKryo = Methods_RMQ.getKryo();
                initedOdomkryo = true;
            }
//            Odometry_ odometry_ = (Odometry_) Methods_RMQ.deSerialize(bytes, Odometry_.class);
            Odometry_ odometry_ = (Odometry_) Methods_RMQ.deSerialize(odometrySchemeKryo, bytes);
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
        private Kryo scanSchemeKryo;
        private boolean initedScankryo = false;
        @Override
        public List<Object> deserialize(byte[] bytes) {
            if (!initedScankryo) {
                scanSchemeKryo = Methods_RMQ.getKryo();
                initedScankryo = true;
            }
//            PointCloud2_ data = (PointCloud2_) Methods_RMQ.deSerialize(bytes, PointCloud2_.class);
            PointCloud2_ data = (PointCloud2_) Methods_RMQ.deSerialize(scanSchemeKryo, bytes);
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
        private Kryo particleSchemeKryo;
        private boolean initedkryo = false;
        @Override
        public List<Object> deserialize(byte[] bytes) {
            if (!initedkryo) {
                particleSchemeKryo = Methods_RMQ.getKryo();
                initedkryo = true;
            }
//            PoseArray_ data = (PoseArray_) Methods_RMQ.deSerialize(bytes, PoseArray_.class);
            PoseArray_ data = (PoseArray_) Methods_RMQ.deSerialize(particleSchemeKryo, bytes);
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
            PoseShareMsg_ data = (PoseShareMsg_) Methods_RMQ.deSerialize(bytes, PoseShareMsg_.class);
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
            BaseConfig_ data = (BaseConfig_) Methods_RMQ.deSerialize(bytes, BaseConfig_.class);
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
