package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class Vector4d_ implements Serializable {
    private double x;
    private double y;
    private double z;
    private double w;

    public Vector4d_() {
    }

    public Vector4d_(Vector4d_ v4d) {
        x = v4d.getX();
        y = v4d.getY();
        z = v4d.getZ();
        w = v4d.getW();
    }

    public Vector4d_(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public void setW(double w) {
        this.w = w;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getW() {
        return w;
    }

    public Vector4d_ copy() {
        return new Vector4d_(x, y, z, w);

    }

    public double length() {
        return Math.sqrt(x * x + y * y + z * z + w * w);

    }

    @Override
    public String toString() {
        return "("
                + x +
                "," + y +
                "," + z +
                "," + w +
                ')';
    }

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }
}
