package cgl.iotrobots.collavoid.commons;

import com.fasterxml.jackson.databind.ObjectMapper;
import nav_msgs.Odometry;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class Odometry_ implements Serializable {

    private Header_ Header;

    private String ChildFrameId;

    private Pose_ Pose;

    private Twist_ Twist;

    public Odometry_(Odometry odometry) {
        Header = new Header_(odometry.getHeader().getFrameId(),
                odometry.getHeader().getStamp().nsecs);

        ChildFrameId = odometry.getChildFrameId();
        Pose = new Pose_(new Vector3d_(
                odometry.getPose().getPose().getPosition().getX(),
                odometry.getPose().getPose().getPosition().getY(),
                odometry.getPose().getPose().getPosition().getZ()
        ),
                new Vector4d_(
                        odometry.getPose().getPose().getOrientation().getX(),
                        odometry.getPose().getPose().getOrientation().getY(),
                        odometry.getPose().getPose().getOrientation().getZ(),
                        odometry.getPose().getPose().getOrientation().getW()
                ));

        Twist = new Twist_(
                new Vector3d_(
                        odometry.getTwist().getTwist().getAngular().getX(),
                        odometry.getTwist().getTwist().getAngular().getY(),
                        odometry.getTwist().getTwist().getAngular().getZ()
                ),
                new Vector3d_(
                        odometry.getTwist().getTwist().getLinear().getX(),
                        odometry.getTwist().getTwist().getLinear().getY(),
                        odometry.getTwist().getTwist().getLinear().getZ()
                ));

        Twist.setHeader(Header);
    }

    public void setHeader(Header_ header) {
        Header = header;
    }

    public void setHeader(String frameId, long stamp) {
        Header.setFrameId(frameId);
        Header.setStamp(stamp);
    }

    public void setChildFrameId(String childFrameId) {
        ChildFrameId = childFrameId;
    }

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }

}
