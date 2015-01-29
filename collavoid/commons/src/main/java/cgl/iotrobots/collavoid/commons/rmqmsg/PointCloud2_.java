package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class PointCloud2_ implements Serializable {

    private String id;
    private Header_ Header = new Header_();
    private int Width;
    private int Height;
    private double[] Data;
    private int Dimension;


    public Header_ getHeader() {
        return Header;
    }

    public double[] getData() {
        return Data;
    }

    public int getDimension() {
        return Dimension;
    }

    public String getId() {
        return id;
    }

    public int getHeight() {
        return Height;
    }

    public int getWidth() {
        return Width;
    }

    public void setHeader(Header_ header) {
        Header = header;
    }

    public void setWidth(int width) {
        Width = width;
    }

    public void setHeight(int height) {
        Height = height;
    }

    public void setDimension(int dimension) {
        Dimension = dimension;
    }

    public void setData(double[] data) {
        Data = data;
    }

    public void setId(String id) {
        this.id = id;
    }

    public PointCloud2_ copy() {
        PointCloud2_ pointCloud2_ = new PointCloud2_();
        pointCloud2_.setHeader(Header.copy());
        double[] data = new double[Data.length];
        System.arraycopy(Data, 0, data, 0, Data.length);
        pointCloud2_.setData(data);
        pointCloud2_.setDimension(Dimension);
        pointCloud2_.setHeight(Height);
        pointCloud2_.setWidth(Width);
        return pointCloud2_;
    }

}
