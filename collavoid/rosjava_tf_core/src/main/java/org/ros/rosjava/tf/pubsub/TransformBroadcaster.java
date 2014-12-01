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

import java.util.ArrayList;

import com.google.api.client.repackaged.com.google.common.base.Preconditions;
import geometry_msgs.Transform;

import geometry_msgs.TransformStamped;
import org.ros.node.ConnectedNode;

import org.ros.node.topic.Publisher;
import org.ros.message.Time;
import tf2_msgs.TFMessage;


/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 * @brief This is a simple class to provide sendTransform() (akin to rospy and roscpp versions); it handles creation of publisher and advertising for you.
 */

public class TransformBroadcaster {

	protected ConnectedNode connectednode;
	protected Publisher<TFMessage> pub;

	public TransformBroadcaster(ConnectedNode connectednode) {
		this.connectednode = connectednode;
		advertise();
	}
	
	protected void advertise() {
		Preconditions.checkNotNull(connectednode);
		this.pub = connectednode.newPublisher("tf", TFMessage._TYPE);
		this.pub.setLatchMode(true);
        connectednode.getLog().debug("TransformBroadcaster advertised on /tf.");
	}

	public void sendTransform(
									String parentFrame, String childFrame,
                                    Time now,
									double v_x, double v_y, double v_z,
									double q_x, double q_y, double q_z, double q_w // quaternion
									) {
		
		// WARN if quaternion not normalized, and normalize it
		// WARN if time is in the future, or otherwise looks funky (negative? more than a year old?)
		Preconditions.checkNotNull(connectednode);

		TransformStamped txMsg =  connectednode.getTopicMessageFactory().newFromType(TransformStamped._TYPE);
		txMsg.getHeader().setStamp(now);
		txMsg.getHeader().setFrameId(parentFrame);
		txMsg.setChildFrameId(childFrame);


		// TODO: invert transform, if it is not cool (have to add tfTree here, then...)
        Transform transform= connectednode.getTopicMessageFactory().newFromType(Transform._TYPE);
        transform.getTranslation().setX(v_x);
        transform.getTranslation().setY(v_y);
        transform.getTranslation().setZ(v_z);
        transform.getRotation().setX(q_x);
        transform.getRotation().setY(q_y);
        transform.getRotation().setZ(q_z);
        transform.getRotation().setW(q_w);

        txMsg.setTransform(transform);

        TFMessage msg = pub.newMessage();
        msg.setTransforms(new ArrayList<TransformStamped>(1));
		msg.getTransforms().add(txMsg);

		pub.publish(msg);
	}

}
