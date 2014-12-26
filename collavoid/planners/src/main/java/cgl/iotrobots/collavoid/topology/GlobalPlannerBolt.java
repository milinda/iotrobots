package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.rmqmsg.PoseStamped_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Created by hjh on 12/23/14.
 */
public class GlobalPlannerBolt extends BaseRichBolt {
    private OutputCollector outputCollector;

    @Override
    public void execute(Tuple input) {
        Object time = input.getValueByField(Constant_storm.Fields.TIME_FIELD);
        Object sensorId = input.getValueByField(Constant_storm.Fields.SENSOR_ID_FIELD);
        Object startGoal = input.getValueByField(Constant_storm.Fields.START_GOAL_FIELD);
        if (!(startGoal instanceof HashMap)) {
            throw new IllegalArgumentException("Start and goal should be Map.");
        }
        Map<String, PoseStamped_> startGoalMap = (HashMap<String, PoseStamped_>) startGoal;
        List<PoseStamped_> plan = new ArrayList<PoseStamped_>();
        makePlan(startGoalMap.get("start"), startGoalMap.get("goal"), plan);
        List<Object> emit = new ArrayList<Object>();
        emit.add(time);
        emit.add(sensorId);
        emit.add(plan);
        outputCollector.emit(emit);
        outputCollector.ack(input);
    }

    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        outputCollector = collector;
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(
                Constant_storm.Fields.TIME_FIELD,
                Constant_storm.Fields.SENSOR_ID_FIELD,
                Constant_storm.Fields.PLAN_FIELD
        ));
    }

    public void makePlan(final PoseStamped_ start, final PoseStamped_ goal, List<PoseStamped_> plan) {
        //start and goal are supposed not to change

        plan.clear();
        plan.add(start);
        double x, y, dir_x, dir_y;
        dir_x = goal.getPose().getPosition().getX() - start.getPose().getPosition().getX();
        dir_y = goal.getPose().getPosition().getY() - start.getPose().getPosition().getY();
        double length = Math.sqrt(dir_y * dir_y + dir_x * dir_x);
        dir_x /= length;
        dir_y /= length;
        x = start.getPose().getPosition().getX() + 0.1 * dir_x;
        y = start.getPose().getPosition().getY() + 0.1 * dir_y;

        while (Math.abs(x - goal.getPose().getPosition().getX()) > 0.2 || Math.abs(y - goal.getPose().getPosition().getY()) > 0.2) {
            PoseStamped_ pose = new PoseStamped_();
            pose.getPose().getPosition().setX(x);
            pose.getPose().getPosition().setY(y);
            pose.getHeader().setFrameId(goal.getHeader().getFrameId());
            pose.getHeader().setStamp(goal.getHeader().getStamp());
            plan.add(pose);
            x += 0.1 * dir_x;
            y += 0.1 * dir_y;
        }
        plan.add(goal);

    }
}
