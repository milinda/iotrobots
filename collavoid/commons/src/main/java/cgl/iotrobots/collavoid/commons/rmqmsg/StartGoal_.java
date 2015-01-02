package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

/**
 * Created by hjh on 1/1/15.
 */
public class StartGoal_ implements Serializable {
    private PoseStamped_ start;
    private PoseStamped_ goal;

    public PoseStamped_ getGoal() {
        return goal;
    }

    public PoseStamped_ getStart() {
        return start;
    }

    public void setGoal(PoseStamped_ goal) {
        this.goal = goal;
    }

    public void setStart(PoseStamped_ start) {
        this.start = start;
    }

    @Override
    public String toString() {
        return "{start: " + start.toString() + ";" + "goal: " + goal.toString() + "}";
    }

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }
}
