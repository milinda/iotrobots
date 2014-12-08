package cgl.iotrobots.slam.streaming.msgs;

import java.util.ArrayList;
import java.util.List;

public class ParticleAssignments {
    List<ParticleAssignment> assignments = new ArrayList<ParticleAssignment>();

    public ParticleAssignments() {
    }

    public List<ParticleAssignment> getAssignments() {
        return assignments;
    }

    public void setAssignments(List<ParticleAssignment> assignments) {
        this.assignments = assignments;
    }

    public void addAssignment(ParticleAssignment assignment) {
        assignments.add(assignment);
    }
}
