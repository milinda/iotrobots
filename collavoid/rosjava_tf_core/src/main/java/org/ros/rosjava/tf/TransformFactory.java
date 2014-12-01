package org.ros.rosjava.tf;
import java.util.ArrayList;
import java.util.Collection;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import geometry_msgs.*;
import geometry_msgs.Transform;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import tf2_msgs.TFMessage;


public class TransformFactory {
    public static MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
    public static MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);
	
	public static Collection<StampedTransform> fromTfMessage(TFMessage msg) {
		ArrayList<StampedTransform> transforms = new ArrayList<StampedTransform>(msg.getTransforms().size());
		for(int i = 0; i < msg.getTransforms().size(); i++) {
			transforms.add(TransformFactory.msg2transform(msg.getTransforms().get(i)));
		}
		return transforms;
	}

	public static StampedTransform msg2transform(TransformStamped msg) {
		return new StampedTransform(
							msg.getHeader().getStamp().totalNsecs(),
							msg.getHeader().getFrameId(),
							msg.getChildFrameId(),
							msg2vector(msg.getTransform().getTranslation()),
							msg2quaternion(msg.getTransform().getRotation())
						);
	}
	
	public static Quat4d msg2quaternion(Quaternion q) {
		return new Quat4d(q.getX(), q.getY(), q.getZ(), q.getW());
	}

	public static Vector3d msg2vector(Vector3 v) {
		return new Vector3d(v.getX(), v.getY(), v.getZ());
	}

	public static TransformStamped tx2msg(StampedTransform tx) {


		TransformStamped msg = messageFactory.newFromType(TransformStamped._TYPE);
        msg.getHeader().setFrameId(tx.parentFrame);
        msg.setChildFrameId(tx.childFrame);
        msg.getHeader().setStamp(Time.fromNano(tx.timestamp));

        Transform transform = messageFactory.newFromType(Transform._TYPE);
        transform.setTranslation(vector2msg(tx.translation));
        transform.setRotation(quaternion2msg(tx.rotation));
        msg.setTransform(transform);

//		msg.header.frame_id = tx.parentFrame;
//		msg.child_frame_id = tx.childFrame;
//		msg.header.stamp = org.ros.message.Time.fromNano(tx.timestamp);
//		msg.transform = new org.ros.message.geometry_msgs.Transform();
//		msg.transform.translation = vector2msg(tx.translation);
//		msg.transform.rotation = quaternion2msg(tx.rotation);
		return msg;
	}
	
	public static Vector3 vector2msg(Vector3d v) {
		Vector3 msg = messageFactory.newFromType(Vector3._TYPE);
		msg.setX(v.getX());
        msg.setY(v.getY());
        msg.setZ(v.getZ());
//		msg.y = v.y;
//		msg.z = v.z;
		return msg;
	}
	
	public static Quaternion quaternion2msg(Quat4d q) {
		//org.ros.message.geometry_msgs.Quaternion msg = new org.ros.message.geometry_msgs.Quaternion();
        Quaternion msg = messageFactory.newFromType(Quaternion._TYPE);
        msg.setX(q.getX());
        msg.setY(q.getY());
        msg.setZ(q.getZ());
        msg.setW(q.getW());
//		msg.x = q.x;
//		msg.y = q.y;
//		msg.z = q.z;
//		msg.w = q.w;
		return msg;
	}
	
}
