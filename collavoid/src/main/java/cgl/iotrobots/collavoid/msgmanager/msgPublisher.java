package cgl.iotrobots.collavoid.msgmanager;

/**
* Created by hjh on 10/29/14.
*/
import cgl.iotrobots.collavoid.agent.agent;
import cgl.iotrobots.collavoid.utils.obstacle;
import com.sun.xml.internal.bind.v2.TODO;
import org.ros.node.topic.Publisher;

import java.util.Vector;

public class msgPublisher {




    public static void publishObstacleLines(final Vector<obstacle> obstacles_lines, String base_frame, String name_space, Publisher line_pub){


    }


    public static void publishMePosition(agent me, String base_frame, String name_space, Publisher me_pub){

    }


    public static void publishNeighborPositions(Vector<agent> neighbors, String base_frame, String name_space, Publisher neighbors_pub){

    }



}
