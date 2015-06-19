package cgl.iotrobots.collavoid.commons.rmqmsg;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

public class PoseArray_ implements Serializable {

    private String id;
    private Header_ Header = new Header_();
    private List<Pose_> Poses = new ArrayList<Pose_>();


    public Header_ getHeader() {
        return Header;
    }

    public List<Pose_> getPoses() {
        return Poses;
    }

    public String getId() {
        return id;
    }

    public void setHeader(Header_ header) {
        Header = header;
    }

    public void setId(String id) {
        this.id = id;
    }

    public void setPoses(List<Pose_> poses) {
        Poses = poses;
    }

    public PoseArray_ copy() {
        PoseArray_ poseArray_ = new PoseArray_();
        poseArray_.setHeader(Header.copy());
        for (Pose_ pose_ : Poses)
            poseArray_.getPoses().add(pose_.copy());
        return poseArray_;
    }

}
