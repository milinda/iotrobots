package cgl.iotrobots.collavoid.topology;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.planners.Line;
import cgl.iotrobots.collavoid.commons.planners.Methods_Planners;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by hjh on 12/25/14.
 */
public class AddNHConstraintsBolt extends BaseBasicBolt {
    private Agent agent;
    private List<Line> nhConstLines = new ArrayList<Line>();
    private int seq = 0;

    @Override
    public void execute(Tuple input, BasicOutputCollector collector) {
        agent = (Agent) input.getValueByField(Constant_storm.Fields.AGENT_FIELD);
        nhConstLines.clear();
        addNHConstraints();
        collector.emit(new Values(input.getValue(0), input.getValue(1), nhConstLines, seq++));
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.Fields.TIME_FIELD,
                Constant_storm.Fields.SENSOR_ID_FIELD,
                Constant_storm.Fields.ADDORCA_LINES_FIELD,
                Constant_storm.Fields.SEQUENCE_FIELD));
    }

    private void addNHConstraints() {
        double min_error = agent.minErrorHolo;
        double max_error = agent.maxErrorHolo;
        double error = max_error;
        double v_max_ang = vMaxAng();

        //ROS_ERROR("v_max_ang %.2f", v_max_ang);

        if (agent.min_dist < 2.0 * agent.footprint_radius_ + agent.cur_loc_unc_radius_) {
            error = (max_error - min_error) /
                    (Math.pow(2 * (agent.footprint_radius_ + agent.cur_loc_unc_radius_), 2)) *
                    Math.pow(agent.min_dist, 2) + min_error; // how much error do i allow?
            //ROS_DEBUG("Error = %f", error);
            if (agent.min_dist < 0) {
                error = min_error;
                // ROS_DEBUG("%s I think I am in collision", me_->getId().c_str());
            }
        }
        agent.cur_allowed_error_ = 1.0 / 3.0 * agent.cur_allowed_error_ + 2.0 / 3.0 * error;
        //ROS_ERROR("error = %f", cur_allowed_error_);
        double speed_ang = Math.atan2(agent.prefVelociy.getY(), agent.prefVelociy.getX());
        double dif_ang = Methods_Planners.shortest_angular_distance(agent.position.getHeading(), speed_ang);
        //calculate possible tracking holomonic robot speed range
        if (Math.abs(dif_ang) > Math.PI / 2.0) { // || cur_allowed_error_ < 2.0 * min_error) {
            double max_track_speed = Methods_Planners.NHORCA.calculateMaxTrackSpeedAngle(
                    agent.time_to_holo_,
                    Math.PI / 2.0,
                    agent.cur_allowed_error_,
                    agent.max_vel_x_,
                    agent.max_vel_th_,
                    v_max_ang);
            // tracking errors are not in velocity space!!!!!!!!!!!!!!
            if (max_track_speed <= 2 * min_error) {
                max_track_speed = 2 * min_error;
            }
            Methods_Planners.NHORCA.addMovementConstraintsDiffSimple(
                    max_track_speed,
                    agent.position.getHeading(),
                    nhConstLines);
        } else {
            Methods_Planners.NHORCA.addMovementConstraintsDiff(
                    agent.cur_allowed_error_,
                    agent.time_to_holo_,
                    agent.max_vel_x_,
                    agent.max_vel_th_,
                    agent.position.getHeading(),
                    v_max_ang,
                    nhConstLines);
        }
        agent.max_speed_x_ = vMaxAng();

    }

    double vMaxAng() {
        //double theoretical_max_v = max_vel_th_ * wheel_base_ / 2.0;
        //return theoretical_max_v - std::abs(base_odom_.twist.twist.angular.z) * wheel_base_/2.0;
        return agent.max_vel_x_; //TODO: fixme
    }
}
