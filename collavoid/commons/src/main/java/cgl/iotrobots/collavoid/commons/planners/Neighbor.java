package cgl.iotrobots.collavoid.commons.planners;

import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Neighbor implements Serializable {
    protected Lock lock;
    public String Name = "robot";
    public String id;
    public boolean holo_robot_;
    protected Odometry_ base_odom_ = new Odometry_();
    // for getting obstacles, but the algorithm uses footprint to calculate velocity, this may be not right
    public double radius;
    public boolean controlled;
    protected List<Vector2> footprint_minkowski = new ArrayList<Vector2>();
    public List<Vector2> footprint_rotated = new ArrayList<Vector2>();
    public long last_seen_;
    public long lastTimeMePublished;
    public long lastTimeControlled;
    //updated robot state
    public Position position = new Position();
    public Vector2 velocity = new Vector2();

    public double controlPeriod = 0.05;
    private long seq;

    public Neighbor() {

    }

    public Neighbor(String ID) {
        this.id = ID;
        lock = new ReentrantLock();
    }

    public Neighbor(String ID, String name) {
        this(ID);
        this.Name = name;
    }

    public String getName() {
        return Name;
    }

    public long getLastSeen() {
        return last_seen_;
    }

    public Odometry_ getBaseOdom() {
        return base_odom_;
    }

    public boolean getHoloRobot() {
        return holo_robot_;

    }

    public Position getPosition() {
        return position;
    }

    public Lock getLock() {
        return lock;
    }

    public List<Vector2> getFootprint_minkowski() {
        return footprint_minkowski;
    }

    public double getControlPeriod() {
        return controlPeriod;
    }

    public double getRadius() {
        return radius;
    }

    public long getSeq() {
        return seq;
    }

    public void setPosition(Position position) {
        this.position = position;
    }

    public void setFootprint_rotated(List<Vector2> footprint_rotated) {
        this.footprint_rotated = footprint_rotated;
    }

    public void setFootprint_minkowski(List<Vector2> footprint_minkowski) {
        this.footprint_minkowski = footprint_minkowski;
    }

    public void setVelocity(Vector2 velocity) {
        this.velocity = velocity;
    }

    public void setHolo_robot_(boolean holo_robot_) {
        this.holo_robot_ = holo_robot_;
    }

    public void setControlled(boolean controlled) {
        this.controlled = controlled;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public void setLast_seen_(long last_seen_) {
        this.last_seen_ = last_seen_;
    }

    public void setControlPeriod(double controlPeriod) {
        this.controlPeriod = controlPeriod;
    }

    public void setBase_odom_(Odometry_ base_odom_) {
        this.base_odom_ = base_odom_;
        this.last_seen_ = base_odom_.getHeader().getStamp();
    }

    public void setName(String name) {
        Name = name;
    }

    public void setSeq(long seq) {
        this.seq = seq;
    }

    public Neighbor copy() {
        Neighbor res = new Neighbor(id, Name);
        res.setControlPeriod(controlPeriod);
        res.setControlled(controlled);
        res.setHolo_robot_(holo_robot_);
        res.setLast_seen_(last_seen_);
        res.setRadius(radius);
        res.setPosition(position.copy());
        res.setVelocity(velocity.copy());
        res.setBase_odom_(base_odom_.copy());
        List<Vector2> footprintMin = new ArrayList<Vector2>();
        List<Vector2> footprintRotated = new ArrayList<Vector2>();
        for (Vector2 vec : footprint_minkowski)
            footprintMin.add(vec.copy());
        for (Vector2 vec : footprint_rotated)
            footprintRotated.add(vec.copy());
        res.setFootprint_minkowski(footprintMin);
        res.setFootprint_rotated(footprintRotated);
        return res;

    }
}
