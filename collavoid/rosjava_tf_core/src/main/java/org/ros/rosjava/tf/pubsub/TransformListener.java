/* 
 * Copyright 2011 Heuristic Labs, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *   
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.ros.rosjava.tf.pubsub;

import java.util.Collection;
import java.util.List;
import java.util.Observable;
import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

//import com.touchgraph.graphlayout.GraphListener;
import geometry_msgs.Twist;
import geometry_msgs.Vector3;
import org.jgrapht.event.GraphListener;
import org.ros.message.Duration;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.*;

import std_msgs.Header;

import com.google.api.client.repackaged.com.google.common.base.Preconditions;
import tf2_msgs.TFMessage;

/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 */
//time is in totalNsecs
public class TransformListener extends Observable {

    protected ConnectedNode node;
    protected TransformTree tfTree;

    public TransformListener(ConnectedNode node) {
        this.node = node;
        this.tfTree = new TransformTree();
        subscribe();
    }

    private void subscribe() {
        Preconditions.checkNotNull(node);
        Subscriber<TFMessage> subscriber = node.newSubscriber("tf", TFMessage._TYPE);//or /tf/tfMessage
        subscriber.addMessageListener(new MessageListener<TFMessage>() {
            @Override
            public void onNewMessage(final TFMessage message) {
                Collection<StampedTransform> transforms = TransformFactory.fromTfMessage(message);
//                if (message.getTransforms().get(0).getHeader().getFrameId().equals("robot7_odometry")&&
//                        message.getTransforms().get(0).getChildFrameId().equals("robot7_base"))
//                System.out.println("time: "+message.getTransforms().get(0).getHeader().getStamp().totalNsecs()
//                        +"; parent frame: "+message.getTransforms().get(0).getHeader().getFrameId()+
//                        "; child frame: "+message.getTransforms().get(0).getChildFrameId());
                tfTree.add(transforms);
            }
        },20);
    }

    public void addListener(GraphListener<String, TransformBuffer> listener) {
        tfTree.getGraph().addGraphListener(listener);
    }

    public TransformTree getTree() {
        return tfTree;
    }

    public Transform3D transform(String target_frame, String source_frame) {
        Transform3D tf=null;
        Transform transform;
        if (tfTree.canTransform(source_frame, target_frame)) {
            transform = tfTree.lookupMostRecent(source_frame, target_frame);
            if (transform==null){
                return tf;
            }
            tf=new Transform3D(transform.rotation, transform.translation, 1);
            return tf;
        }
        if (tfTree.canTransform(target_frame, source_frame)) {
            transform = tfTree.lookupMostRecent(target_frame, source_frame);
            if (transform==null)
                return tf;
            transform.rotation.setW(-transform.rotation.getW());
            transform.translation.scale(-1);
            tf = new Transform3D(transform.rotation,transform.translation, 1);
            return tf;
        }
        return tf;
    }

    public Transform3D transform(String target_frame, String source_frame, long t) {
        Transform3D tf=null;
        Transform transform;
        if (tfTree.canTransform(source_frame, target_frame, t)) {
            transform = tfTree.lookupTransformBetween(source_frame, target_frame, t);
            tf = new Transform3D(transform.rotation, transform.translation, 1);
            return tf;
        }
        if (tfTree.canTransform(target_frame, source_frame, t)) {
            transform = tfTree.lookupTransformBetween(target_frame, source_frame, t);
            transform.rotation.setW(-transform.rotation.getW());
            transform.translation.scale(-1);
            tf = new Transform3D(transform.rotation,transform.translation, 1);
            return tf;
        }
        return tf;
    }

    public Transform3D transform(String target_frame, String source_frame, int cnt) {
        Transform3D tf3D=null;
        int i = 0;
        while (i < cnt) {
            tf3D=transform(target_frame, source_frame);
            if (tf3D != null)
                break;
            i++;
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        return tf3D;
    }

    public Transform3D transform(String target_frame, String source_frame, long t,int cnt) {
        Transform3D tf3D=null;
        int i = 0;
        while (i < cnt) {
            tf3D=transform(target_frame, source_frame,t);
            if (tf3D != null)
                break;
            i++;
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        return tf3D;
    }

    //not use
    private Transform3D transform(String target_frame, String source_frame, long t, Duration dur_m) {
        Transform3D tf3D=null;
        int i = 0;
        int time_step = dur_m.nsecs / 10000000;
        if (time_step < 10)
            time_step = 10;
        while (i < 10) {
            tf3D=transform(target_frame, source_frame, t);
            if (tf3D != null)
                break;
            i++;
            try {
                Thread.sleep(time_step);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        return tf3D;
    }


    public boolean transformTwist(String target_frame, String source_frame, Twist source_twist, Twist dest_twist) {
        Transform3D tf3d;
        Point3d ptLinear = new Point3d();
        Point3d ptAngular = new Point3d();
        Vector3ToPoint3d(source_twist.getLinear(), ptLinear);
        Vector3ToPoint3d(source_twist.getAngular(), ptAngular);

        Vector3 vcLinear = node.getTopicMessageFactory().newFromType(Vector3._TYPE);
        Vector3 vcAngular = node.getTopicMessageFactory().newFromType(Vector3._TYPE);

        // try transform 5 times as sometimes transform may fail
        // FIXME: sometimes transform fails
        tf3d=transform(target_frame, source_frame,5);


        if (tf3d == null)
            return false;
        // in velocity space no translation in velocity
        tf3d.setTranslation(new Vector3d(0,0,0));
        // the robot publishes transforms that has reversed parent and children, so need to reverse transform
        Quat4d tfrq=new Quat4d();
        tf3d.get(tfrq);
        tfrq.setW(-tfrq.getW());
        tf3d.set(tfrq);

        tf3d.transform(ptLinear);
        tf3d.transform(ptAngular);
        Point3dToVector3(ptLinear, vcLinear);
        Point3dToVector3(ptAngular, vcAngular);
        dest_twist.setLinear(vcLinear);
        dest_twist.setAngular(vcAngular);
        return true;
    }

    //not used
    public boolean transformPoint3ds(String target_frame, String source_frame, List<Point3d> points, long t,Duration dur_m) {
        Transform3D tf3d;
        tf3d=transform(target_frame, source_frame, t,dur_m);
        if (tf3d == null)
            return false;

        for (int i = 0; i < points.size(); i++) {
                tf3d.transform(points.get(i));
        }
        return true;
    }





    private void Vector3ToPoint3d(Vector3 in, Point3d out) {
        out.set(in.getX(), in.getY(), in.getZ());
    }

    private void Point3dToVector3(Point3d in, Vector3 out) {
        out.setX(in.getX());
        out.setY(in.getY());
        out.setZ(in.getZ());
    }

}

