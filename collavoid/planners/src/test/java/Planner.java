import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.planners.GlobalPlanner;
import cgl.iotrobots.collavoid.planners.LocalPlanner;
import com.rabbitmq.client.Address;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Planner {

    private volatile boolean running;

    private Thread pubVelThread = null;

    private RMQContext velContext;

    private long lastPublished;

    private double ControlPeriod;

    private LocalPlanner localPlanner;

    private GlobalPlanner globalPlanner;

    private PoseStamped_ start = new PoseStamped_();

    private PoseStamped_ goal = new PoseStamped_();

    private List<PoseStamped_> plan = new ArrayList<PoseStamped_>();

    public Planner(String name,
                   Address[] addresses,
                   String url) {
        globalPlanner = new GlobalPlanner();
        localPlanner = new LocalPlanner(name, addresses, url);
        initialize();
    }

    private void initialize() {
        ControlPeriod = localPlanner.getAgent().getControlPeriod();
        lastPublished = System.currentTimeMillis();
        velContext = localPlanner.getAgent().getRmqMsgManager().getRMQContexts().get(Constant.KEY_VELOCITY_CMD);
    }

    public PoseStamped_ getStart() {
        return start;
    }

    public PoseStamped_ getGoal() {
        return goal;
    }

    public void setStartGoal(PoseStamped_ start, PoseStamped_ goal) {
        this.start = start;
        this.goal = goal;
        globalPlanner.makePlan(start, goal, plan);
        localPlanner.setPlan(plan);
    }

    public void run() {
        running = true;
        pubVelThread = new Thread(new Runnable() {
            @Override
            public void run() {

                Twist_ cmd_vel = new Twist_();
                while (running) {
                    if (System.currentTimeMillis() - lastPublished > (long) ControlPeriod * 1000) {
                        lastPublished = System.currentTimeMillis();
                        if (localPlanner.computeVelocityCommands(cmd_vel)) {
//                            try {
//                                System.out.println(cmd_vel);
//                                Methods_RMQ.publishMsg(velContext, cmd_vel.toJSON());
//                            } catch (IOException e) {
//                                e.printStackTrace();
//                            }
                        }
                    }
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                        running = false;
                    }
                }
            }
        });
        pubVelThread.start();
    }

    public void stop() {
        if (localPlanner != null) {
            localPlanner.stop();
        }
        if (pubVelThread != null) {
            running = false;
            try {
                pubVelThread.join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}
