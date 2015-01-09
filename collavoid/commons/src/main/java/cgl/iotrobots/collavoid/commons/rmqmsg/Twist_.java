package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class Twist_ implements Serializable {

    private Vector3d_ Angular = new Vector3d_();

    private Vector3d_ Linear = new Vector3d_();

    public Twist_() {
    }

    public void setAngular(Vector3d_ angular) {
        this.Angular = new Vector3d_(angular);
    }

    public void setLinear(Vector3d_ linear) {
        this.Linear = new Vector3d_(linear);
    }

    public Vector3d_ getAngular() {
        return Angular;
    }

    public Vector3d_ getLinear() {
        return Linear;
    }

    public Twist_ copy() {
        Twist_ twist_ = new Twist_();
        twist_.setAngular(Angular.copy());
        twist_.setLinear(Linear.copy());
        return twist_;
    }

    @Override
    public String toString() {
        return "{" +
                "angular=" + Angular +
                ", linear=" + Linear +
                '}';
    }


}
