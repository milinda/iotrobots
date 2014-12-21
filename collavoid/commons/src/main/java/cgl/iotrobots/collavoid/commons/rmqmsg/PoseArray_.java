package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

public class PoseArray_ implements Serializable {
    private Header_ Header = new Header_();

    private List<Pose_> Poses = new ArrayList<Pose_>();

    public PoseArray_() {

    }

    public Header_ getHeader() {
        return Header;
    }

    public List<Pose_> getPoses() {
        return Poses;
    }

    public void setHeader(Header_ header) {
        Header = header;
    }

    public void setPoses(List<Pose_> poses) {
        Poses = poses;
    }

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }
}
