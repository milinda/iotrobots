package testTopology;

import backtype.storm.spout.Scheme;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.io.IOException;
import java.util.List;

public class Schemes {
    public static class OdometryScheme implements Scheme {
        @Override
        public List<Object> deserialize(byte[] bytes) {
            return new Values(deserializeObject(bytes));
        }

        public static Odometry_ deserializeObject(byte[] body) {
            return (Odometry_) Methods_RMQ.deserialize(body, Odometry_.class);
        }

        @Override
        public Fields getOutputFields() {
            return new Fields(Constant_storm.FIELDS.ODOMETRY_FIELD);
        }
    }

    public static class ScanScheme implements Scheme {
        @Override
        public List<Object> deserialize(byte[] bytes) {
            return new Values(deserializeObject(bytes));
        }

        public static PointCloud2_ deserializeObject(byte[] body) {
            return (PointCloud2_) Methods_RMQ.deserialize(body, PointCloud2_.class);
        }

        @Override
        public Fields getOutputFields() {
            return new Fields(Constant_storm.FIELDS.SCAN_FIELD);
        }
    }

    public static class ParticleScheme implements Scheme {
        @Override
        public List<Object> deserialize(byte[] bytes) {
            return new Values(deserializeObject(bytes));
        }

        public static PoseArray_ deserializeObject(byte[] body) {
            return (PoseArray_) Methods_RMQ.deserialize(body, PoseArray_.class);
        }

        @Override
        public Fields getOutputFields() {
            return new Fields(Constant_storm.FIELDS.POSE_ARRAY_FIELD);
        }
    }

    public static class PoseShareScheme implements Scheme {
        @Override
        public List<Object> deserialize(byte[] bytes) {
            return new Values(deserializeObject(bytes));
        }

        public static PoseShareMsg_ deserializeObject(byte[] body) {
            return (PoseShareMsg_) Methods_RMQ.deserialize(body, PoseShareMsg_.class);
        }

        @Override
        public Fields getOutputFields() {
            return new Fields(Constant_storm.FIELDS.POSE_SHARE_FIELD);
        }
    }

    public static class startGoalScheme implements Scheme {
        @Override
        public List<Object> deserialize(byte[] bytes) {
            return new Values(deserializeObject(bytes));
        }

        public static BaseConfig_ deserializeObject(byte[] body) {
            return (BaseConfig_) Methods_RMQ.deserialize(body, BaseConfig_.class);
        }

        @Override
        public Fields getOutputFields() {
            return new Fields(Constant_storm.FIELDS.START_GOAL_FIELD);
        }
    }
}
