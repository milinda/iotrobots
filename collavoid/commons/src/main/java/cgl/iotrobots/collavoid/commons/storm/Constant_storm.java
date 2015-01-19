package cgl.iotrobots.collavoid.commons.storm;

import backtype.storm.tuple.Fields;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;

import java.util.HashMap;
import java.util.Map;

public abstract class Constant_storm {
    public abstract static class FIELDS {
        //general
        public static final String TIME_FIELD="time";
        public static final String SENSOR_ID_FIELD="sensorID";
        //global planner
        public static final String START_GOAL_FIELD="startGoal";
        public static final String BASE_CONFIG_FIELD="baseConfig";//in streaming
        public static final String PLAN_FIELD="plan";
        //local planner
        public static final String PREFERRED_VELOCITY_FIELD="preferredVelocity";
        public static final String VELOCITY_COMMAND_FIELD="velocityCommand";
        public static final String RESET_FIELD="reset";
        //odometry
        public static final String ODOMETRY_FIELD="odometry";
        //scan get obstacles
        public static final String SCAN_FIELD="scan";
        public static final String ALL_AGENTS_FIELD ="agents";
        public static final String OBSTACLE_FIELD="obstacles";
        //pose share spout
        public static final String POSE_SHARE_FIELD="poseShare";        
        // get minkowski footprint
        public static final String FOOTPRINT_OWN_FIELD="footprintOwn";
        public static final String POSE_ARRAY_FIELD="poseArray";
        public static final String FOOTPRINT_MINKOWSK_FIELD="minkowskiFootprint";
        // compute vo lines
        public static final String AGENT_FIELD="agent";
        public static final String ACC_LINES_FIELD ="accLines";// to joint them need to separate each type
        public static final String NH_LINES_FIELD ="nhLines";
        public static final String OBSTACLE_LINES_FIELD ="obstacleLines";// for orca
        public static final String AGENT_VO_FIELD="agentVO";
        public static final String OBSTACLE_VO_FIELD="obstacleVO";
        public static final String SEQUENCE_FIELD="sequence";        
        
        public static final Fields JOIN_FIELDS=new Fields(
                AGENT_FIELD,
                ACC_LINES_FIELD,
                NH_LINES_FIELD,
                OBSTACLE_LINES_FIELD,
                AGENT_VO_FIELD,
                OBSTACLE_VO_FIELD
        );
        public static final String ORCA_LINES_FIELD="orcaLines";
        public static final String VOS_FIELD="vos";
        //agent state
        public static final String NEIGHBORS_FIELD="neighbors";
        // timer
        public static final String TIMER_TICK_FIELD ="timerTick";
    }

    public abstract class Streams{
        //local planner
        public static final String PREFERRED_VELOCITY_STREAM="preferredVelocity";
        public static final String VELOCITY_COMMAND_STREAM="velocityCommand";
        public static final String FOOTPRINT_OWN_STREAM="footprintStream";
        public static final String PUBLISHME_STREAM="publishMe";
        public static final String CALCULATE_VELOCITY_CMD_STREAM ="calculateVelocity";
        public static final String PUBLISH_ME_TIMER_STREAM="publishMeTimer";
        public static final String CONTROLLER_TIMER_STREAM="controllerTimer";
        public static final String NEIGHBORS_STREAM="neighbors";
        public static final String RESET_STREAM="reset";

    }

    public abstract class Components{
        public static final String GLOBAL_PLANNER_COMPONENT="globalPlanner";
        public static final String ODOMETRY_COMPONENT="odometry";
        public static final String LOCAL_PLANNER_COMPONENT="localPlanner";
        public static final String AGENT_COMPONENT="agent";
        public static final String SCAN_COMPONENT="scan";
        public static final String GET_ALL_AGENTS_COMPONENT ="getAgents";
        public static final String POSE_SHARE_COMPONENT="poseShare";
        public static final String POSE_ARRAY_COMPONENT ="poseArray";
        public static final String GET_OBSTACLES_COMPONENT="getObstacles";
        public static final String ODOMETRY_TRANSFORM_COMPONENT="odometryTransform";
        public static final String BASE_CONFIG_COMPONENT ="baseConfig";// in streaming
        public static final String START_GOAL_COMPONENT="startGoal";// may be deprecated
        public static final String VELOCITY_COMMAND_PUBLISHER_COMPONENT="velCmdPub";
        public static final String VELOCITY_COMPUTE_COMPONENT="velocityCompute";
        public static final String AGENT_STATE_COMPONENT="agentState";
        public static final String TIMER_SPOUT_COMPONENT ="timerSpout";//in streaming
        public static final String TIMER_COMPONENT ="timer";
        public static final String ACC_CONSTRAINTS_COMPONENT="accelerationConstraints";
        public static final String NH_CONSTRAINTS_COMPONENT="nhConstraints";
        public static final String VO_AGENT_COMPONENT="voAgent";
        public static final String VO_OBSTACLE_COMPONENT="voObstacle";
        public static final String VO_LINES_JOIN_COMPONENT="voLineJoin";
        public static final String VO_LINES_COMPUTE_COMPONENT = "voLinesCompute";
        public static final String GET_MINKOWSKI_FOOTPRINT_COMPONENT="getMinkowskiFootprint";
    }
    public static class KeyToComponentMap{
        public final Map<String,String> map=new HashMap<String, String>();
        public KeyToComponentMap(){
            map.put(Constant_msg.KEY_SCAN, Components.SCAN_COMPONENT);
            map.put(Constant_msg.KEY_BASE_CONFIG,Components.BASE_CONFIG_COMPONENT);
            map.put(Constant_msg.KEY_POSE_SHARE,Components.POSE_SHARE_COMPONENT);
            map.put(Constant_msg.KEY_ODOMETRY,Components.ODOMETRY_COMPONENT);
            map.put(Constant_msg.KEY_PARTICLE_CLOUD,Components.POSE_ARRAY_COMPONENT);
        }
    }

}
