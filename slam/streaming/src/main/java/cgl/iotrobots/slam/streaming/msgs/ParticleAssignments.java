package cgl.iotrobots.slam.streaming.msgs;

import java.util.ArrayList;
import java.util.List;

public class ParticleAssignments {
    private List<ParticleAssignment> assignments = new ArrayList<ParticleAssignment>();

    /** If the resampler doesn't re sample there will be no assignments */
    private boolean reSampled = false;

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

    public boolean isReSampled() {
        return reSampled;
    }

    public void setReSampled(boolean reSampled) {
        this.reSampled = reSampled;
    }
}
