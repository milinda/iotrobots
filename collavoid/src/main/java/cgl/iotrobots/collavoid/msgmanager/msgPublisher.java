package cgl.iotrobots.collavoid.msgmanager;


import cgl.iotrobots.collavoid.ROSAgent.Agent;
import cgl.iotrobots.collavoid.utils.obstacle;
import org.ros.node.topic.Publisher;

import java.util.List;

public class msgPublisher {




    public static void publishObstacleLines(final List<obstacle> obstacles_lines, String base_frame, String name_space, Publisher line_pub){


    }


    public static void publishMePosition(Agent me, String base_frame, String name_space, Publisher me_pub){

    }


    public static void publishNeighborPositions(List<Agent> neighbors, String base_frame, String name_space, Publisher neighbors_pub){

    }



}
