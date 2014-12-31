package testTopology;

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
    public static class OdometryScheme implements Scheme {
        @Override
        public List<Object> deserialize(byte[] bytes) {
            return new Values(deserializeObject(bytes));
        }

        public static Odometry_ deserializeObject(byte[] body) {
            try {
                return Serializers.JSONToOdometry_(body);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
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
            try {
                return Serializers.JSONToPointCloud2_(body);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
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
            try {
                return Serializers.JSONToPoseArray_(body);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
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
            try {
                return Serializers.JSONToPoseShareMsg_(body);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
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

        public static Object deserializeObject(byte[] body) {
            return Serializers.deSerialize(body, HashMap.class);
        }

        @Override
        public Fields getOutputFields() {
            return new Fields(Constant_storm.FIELDS.START_GOAL_FIELD);
        }
    }
}
