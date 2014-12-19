package cgl.iotrobots.slam.streaming;

import cgl.iotrobots.slam.streaming.msgs.ParticleAssignment;
import cgl.iotrobots.slam.streaming.msgs.ParticleAssignments;
import cgl.iotrobots.slam.streaming.msgs.ParticleValue;
import junit.framework.TestCase;

import java.util.ArrayList;
import java.util.List;

public class ResamplingBoltTest extends TestCase {
    public void testAssignments() throws Exception {
        ReSamplingBolt bolt = new ReSamplingBolt();
        bolt.prepare(null, null, null);

        ParticleValue v1 = new ParticleValue(0, 0, 2, null, null, 0, 0, 0, 0, null);
        ParticleValue v2 = new ParticleValue(0, 1, 2, null, null, 0, 0, 0, 0, null);
        ParticleValue v3 = new ParticleValue(0, 2, 2, null, null, 0, 0, 0, 0, null);
        ParticleValue v4 = new ParticleValue(1, 3, 2, null, null, 0, 0, 0, 0, null);
        ParticleValue v5 = new ParticleValue(1, 4, 2, null, null, 0, 0, 0, 0, null);
        ParticleValue v6 = new ParticleValue(1, 5, 2, null, null, 0, 0, 0, 0, null);

        bolt.addParticleValue(v1);
        bolt.addParticleValue(v2);
        bolt.addParticleValue(v3);
        bolt.addParticleValue(v4);
        bolt.addParticleValue(v5);
        bolt.addParticleValue(v6);

        List<Integer> indexes = new ArrayList<Integer>();
        indexes.add(0);
        indexes.add(0);
        indexes.add(2);
        indexes.add(3);
        indexes.add(4);
        indexes.add(4);

        ParticleAssignments assignments = bolt.createAssignments(indexes);

        for (ParticleAssignment assignment : assignments.getAssignments()) {
            System.out.format("pre task: %d prev i: %d new task: %d new i %d\n",
                    assignment.getPreviousTask(), assignment.getPreviousIndex(), assignment.getNewTask(), assignment.getNewIndex());;
        }
    }
}
