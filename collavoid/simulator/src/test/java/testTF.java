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
//        AgentNode agentNode=new AgentNode("testTF");
//        ConnectedNode node=agentNode.getNode();
//
//        tfl=new TransformListener(node);
        //test random

//        for (int i = 0; i <1000 ; i++) {
//            System.out.println(utils.getGaussianNoise(0,0.5));
//        }
        Point3d pt=new Point3d(1,0,0);
        Point3d pt1=new Point3d();
        Vector3d tftvec=new Vector3d(0,0,0);
        Quat4d tfrqua=new Quat4d(1,0,0,Math.cos(Math.PI/4));
        Quat4d tfrq=new Quat4d(1,0,0,Math.cos(Math.PI/4));
        Transform3D tf=new Transform3D(tfrqua,tftvec,1);
        Transform3D tft=new Transform3D();
        Transform3D tfr=new Transform3D();

        System.out.println(tfrqua.toString());
        tf.invert();
        tf.get(tftvec);
        System.out.println(tftvec.toString());

        tf.get(tfrqua);
        System.out.println(tfrqua.toString()+"\n");



        tf.set(tftvec);
        tf.set(tfrqua);
        tf.get(tftvec);
        tf.get(tfrqua);
        System.out.println(tftvec.toString());
        System.out.println(tfrqua.toString());

        tf.set(tfrqua,tftvec,1);
        tf.transform(pt,pt1);
        System.out.println(pt1.toString());
        tf.invert();
        tf.transform(pt1);
        System.out.println(pt1.toString());

        tfrqua.normalize();
        tft.set(tftvec);
        tfr.set(tfrqua);

        tft.transform(pt,pt1);
        System.out.println(pt1.toString());
        tfr.transform(pt1,pt1);
        System.out.println(pt1.toString());

        tfr.transform(pt,pt1);
        System.out.println(pt1.toString());
        tft.transform(pt1,pt1);
        System.out.println(pt1.toString());


        tfr.invert();
        tfr.transform(pt,pt1);
        System.out.println(pt1.toString());

        tftvec.scale(-1);
        tfrqua.scale(-1);
        tft.set(tftvec);
        tfr.set(tfrqua);
        tft.transform(pt,pt1);
        System.out.println(pt1.toString());
        tfr.transform(pt,pt1);
        System.out.println(pt1.toString());


    }
}
