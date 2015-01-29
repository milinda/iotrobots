package cgl.iotrobots.collavoid.iotTopology;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.Methods_Planners;
import cgl.iotrobots.collavoid.commons.planners.Parameters;
import cgl.iotrobots.collavoid.commons.planners.Vector2;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Vector3d_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class VelocityComputeBolt extends BaseBasicBolt {
    private Twist_ cmdVel = new Twist_();
    private Vector2 newVelocity = new Vector2();// calculated new holo velocity
    private Agent agent;
    private Logger logger = LoggerFactory.getLogger(VelocityComputeBolt.class);

    @Override
    public void execute(Tuple tuple, BasicOutputCollector basicOutputCollector) {
//        System.out.println("Debug-newv cal:"+System.currentTimeMillis());
        agent = (Agent) Utils.deserialize(tuple.getBinaryByField(Constant_storm.FIELDS.AGENT_FIELD));
        newVelocity = Methods_Planners.ClearPath.calculateClearpathVelocity(
                null,
                agent.voAgents,
                agent.addOrcaLines,
                agent.prefVelociy,
                agent.max_speed_x_,
                agent.useTruancation);
//        logger.warn("{}+++++++++++++++cal new vel {}",agent.Name,newVelocity);
        getVelocityCmd();
        basicOutputCollector.emit(new Values(tuple.getValue(0), tuple.getValue(1), cmdVel.copy()));
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.VELOCITY_COMMAND_FIELD
        ));
    }

    private void getVelocityCmd() {
        double speed_ang = Math.atan2(newVelocity.getY(), newVelocity.getX());
        double dif_ang = Methods_Planners.shortest_angular_distance(agent.position.getHeading(), speed_ang);
//        System.out.println(agent.Name+"************************* dif ang: "+dif_ang+"; speed ang "+speed_ang+"; heading "+agent.position.getHeading());

        Vector3d_ linear = new Vector3d_();
        Vector3d_ angular = new Vector3d_();
        if (!agent.holo_robot_) {
            double vel = Vector2.abs(newVelocity);
            double vstar;

            if (Math.abs(dif_ang) > Parameters.EPSILON)
                vstar = Methods_Planners.NHORCA.calcVstar(vel, dif_ang);//get nonholomonic velocity
            else
                vstar = agent.max_vel_x_;

            linear.setX(Math.min(vstar, vMaxAng()));
            linear.setY(0.0);
            cmdVel.setLinear(linear);

            //ROS_ERROR("dif_ang %f", dif_ang);
            if (Math.abs(dif_ang) > 3.0 * Math.PI / 4.0) {
                angular.setZ(
                        Methods_Planners.sign(
                                agent.base_odom_.getTwist().getAngular().getZ()) *
                                Math.min(Math.abs(dif_ang / agent.time_to_holo_),
                                        agent.max_vel_th_)
                );
                cmdVel.setAngular(angular);
            } else {
                angular.setZ(
                        Methods_Planners.sign(dif_ang) *
                                Math.min(Math.abs(dif_ang / agent.time_to_holo_),
                                        agent.max_vel_th_)
                );
                cmdVel.setAngular(angular);
            }
            //ROS_ERROR("vstar = %.3f", vstar);
        } else {
            //new velocity is caculated in world frame, it has to be transformed to robot base frame
            Vector2 rotated_vel = Vector2.rotateVectorByAngle(newVelocity, -agent.position.getHeading());

            linear.setX(rotated_vel.getX());
            linear.setY(rotated_vel.getY());
            cmdVel.setLinear(linear);
            if (agent.min_dist > 2 * agent.footprint_radius_) {
                angular.setZ(Methods_Planners.sign(dif_ang) * Math.min(Math.abs(dif_ang), agent.max_vel_th_));
                cmdVel.setAngular(angular);
            }
        }
    }

    private double vMaxAng() {
        //double theoretical_max_v = max_vel_th_ * wheel_base_ / 2.0;
        //return theoretical_max_v - std::abs(base_odom_.twist.twist.angular.z) * wheel_base_/2.0;
        return agent.max_vel_x_; //TODO: fixme
    }
}
