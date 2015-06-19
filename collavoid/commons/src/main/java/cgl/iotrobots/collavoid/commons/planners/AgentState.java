package cgl.iotrobots.collavoid.commons.planners;

import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

//for simple topology

public class AgentState implements Serializable {
    public String id;
    public Odometry_ odometry_ = null;
    public List<Neighbor> neighbors = new ArrayList<Neighbor>();
    public List<Obstacle> obstacles = new ArrayList<Obstacle>();
    public List<Vector2> minkowskiFootprint = new ArrayList<Vector2>();

    public AgentState() {

    }

    public AgentState(String id) {
        this.id = id;
    }
}