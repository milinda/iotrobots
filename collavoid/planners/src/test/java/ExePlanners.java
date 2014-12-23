
import cgl.iotrobots.collavoid.commons.planners.Methods_Planners;
import cgl.iotrobots.collavoid.commons.planners.Parameters;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;

import java.util.ArrayList;
import java.util.List;

public class ExePlanners {

    static int robotNb = Parameters.ROBOT_NUMBER;
    static double posRadius = Parameters.POSE_RADIUS;
    static List<Planner> planners = new ArrayList<Planner>();


    public static void main(String[] args) {
        String url = "amqp://localhost:5672";

        for (int i = 0; i < robotNb; i++) {
            PoseStamped_ start = new PoseStamped_();
            PoseStamped_ goal = new PoseStamped_();
            setStartsAndGoals(start, goal, i);
            Planner planner = new Planner("robot" + i + "_rmq", null, url);
            planner.setStartGoal(start, goal);
            planner.run();
            planners.add(planner);
        }

        doShutDownWork();
    }

    //set start point and goal all in ros coordinate
    private static void setStartsAndGoals(PoseStamped_ startStamped, PoseStamped_ goalStamped, int id_int) {
        double step = 2 * Math.PI / robotNb;
        Vector3d_ start = new Vector3d_(
                posRadius * Math.cos(id_int * step),
                posRadius * Math.sin(id_int * step), 0);
        Vector3d_ goal = start.copy();
        goal.scale(-1);

        Vector4d_ oriStart;
        Vector4d_ oriGoal;
        double start_ori = Math.PI + id_int * step;
        oriStart = Methods_Planners.getQuaternion(new Vector3d_(0, 0, 1), start_ori);
        oriGoal = Methods_Planners.getQuaternion(new Vector3d_(0, 0, 1), Math.PI + start_ori);

        Header_ header_ = new Header_();
        header_.setStamp(System.currentTimeMillis());
        header_.setFrameId(Parameters.GLOBAL_FRAME);

        startStamped.getPose().setPosition(start);
        startStamped.getPose().setOrientation(oriStart);
        startStamped.setHeader(header_.copy());

        goalStamped.getPose().setPosition(goal);
        goalStamped.getPose().setOrientation(oriGoal);
        goalStamped.setHeader(header_.copy());
    }

    private static void doShutDownWork() {
        Runtime.getRuntime().addShutdownHook(new Thread() {
            public void run() {
                for (Planner planner : planners)
                    planner.stop();
            }
        });
    }
}
