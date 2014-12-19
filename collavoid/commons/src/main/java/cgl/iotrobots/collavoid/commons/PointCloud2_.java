package cgl.iotrobots.collavoid.commons;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.jboss.netty.buffer.ChannelBuffer;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;
import java.util.List;

public class PointCloud2_ implements Serializable {
    private int With;

    private int Height;

    private ChannelBuffer Data;

    private int Dimension;

    public void setWith(int with) {
        With = with;
    }

    public void setHeight(int height) {
        Height = height;
    }

    public void setDimension(int dimension) {
        Dimension = dimension;
    }

    public void setData(ChannelBuffer data) {
        Data = data;
    }

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }
}
