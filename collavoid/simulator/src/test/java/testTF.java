import org.jgrapht.graph.ListenableDirectedGraph;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.rosjava.tf.Transform;
import org.ros.rosjava.tf.TransformBuffer;
import org.ros.rosjava.tf.pubsub.TransformListener;
import sensor_msgs.PointCloud2;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Set;

/**
 * Created by hjh on 11/30/14.
 */
public class testTF {
    static TransformListener tfl;
    static PointCloud2 pc2;

    public static void main(String[] args){
//        ROSAgentNode agentNode=new ROSAgentNode("testTF");
//        ConnectedNode node=agentNode.getNode();
//
//        tfl=new TransformListener(node);
        //test random

//        for (int i = 0; i <1000 ; i++) {
//            System.out.println(utilsSim.getGaussianNoise(0,0.5));
//        }
        Point3d pt=new Point3d(1,0,0);
        Point3d pt1=new Point3d();
        Vector3d tftvec=new Vector3d(1,1,1);
        Quat4d tfrqua=new Quat4d();

        Transform3D tft=new Transform3D();
        Transform3D tfr=new Transform3D();
        tfr.rotZ(Math.PI);
        tfr.get(tfrqua);

        tft.set(tftvec);
        tft.transform(pt,pt1);
        System.out.println(pt1);
        tftvec.scale(-1);
        tft.set(tftvec);
        tft.transform(pt1);
        System.out.println(pt1+"\n");

        tfr.transform(pt,pt1);
        System.out.println(pt1);

        tfrqua.setW(-tfrqua.getW());
        tfr.set(tfrqua);
        tfr.transform(pt1);
        System.out.println(pt1);


    }
}
