package cgl.iotrobots.collavoid.ROSAgent;

import cgl.iotrobots.collavoid.LocalPlanner.LPutils;
import cgl.iotrobots.collavoid.utils.*;
import geometry_msgs.Point;
import geometry_msgs.PoseStamped;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import std_msgs.Header;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;

import java.util.List;

public class MsgPublisher {
    private ConnectedNode MsgPublisherNode;
    private static final int MAX_POINTS_ = 400;

    public MsgPublisher(ConnectedNode connectedNode) {
        MsgPublisherNode = connectedNode;
    }

    public void publishHoloSpeed(Position pos, Vector2 vel, String base_frame, String name_space, Publisher speed_pub) {
        Marker line_list = MsgPublisherNode.getTopicMessageFactory().newFromType(Marker._TYPE);
        line_list.getHeader().setFrameId(base_frame);
        line_list.getHeader().setStamp(MsgPublisherNode.getCurrentTime());
        line_list.setNs(name_space);
        line_list.setAction(Marker.ADD);
        line_list.getPose().getOrientation().setW(1);
        line_list.setType(Marker.ARROW);
        line_list.getScale().setX(0.01);
        line_list.getScale().setY(0.03);
        line_list.getColor().setR((float)0.3);
        line_list.getColor().setG((float)0.5);
        line_list.getColor().setA(1);
        line_list.setId(0);
        Point p = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
        p.setX(pos.getPos().getX());
        p.setY(pos.getPos().getY());
        p.setZ(0.5);
        line_list.getPoints().add(p);
        Point p1 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
        p1.setX(p.getX() + vel.getX());
        p1.setY(p.getY() + vel.getY());
        p1.setZ(p.getZ());
        line_list.getPoints().add(p1);

        speed_pub.publish(line_list);
    }

    // currently use clear path method vos contain agent vos and obstacle vos
    public void publishVOs(Position pos, List<VO> truncated_vos, boolean use_truncation, String base_frame, String name_space, Publisher vo_pub) {
        Marker line_list = MsgPublisherNode.getTopicMessageFactory().newFromType(Marker._TYPE);
        line_list.getHeader().setFrameId(base_frame);
        line_list.getHeader().setStamp(MsgPublisherNode.getCurrentTime());
        line_list.setNs(name_space);
        line_list.setAction(Marker.ADD);
        line_list.getPose().getOrientation().setW(1);
        line_list.setType(Marker.LINE_LIST);
        line_list.getScale().setX(0.015);
        line_list.getColor().setR(0);
        line_list.getColor().setA(1);
        line_list.setId(0);
        if (!use_truncation) {
            for (int i = 0; i < truncated_vos.size(); i++) {
                Point p = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
                //Apex
                p.setX(pos.getPos().getX() + truncated_vos.get(i).getPoint().getX());
                p.setY(pos.getPos().getY() + truncated_vos.get(i).getPoint().getY());
                line_list.getPoints().add(p);

                Point p1 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
                p1.setX(p.getX() + 3 * truncated_vos.get(i).getLeftLegDir().getX());
                p1.setY(p.getY() + 3 * truncated_vos.get(i).getLeftLegDir().getY());
                line_list.getPoints().add(p1);

                //Apex
                Point p3 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
                p3.setX(pos.getPos().getX() + truncated_vos.get(i).getPoint().getX());
                p3.setY(pos.getPos().getY() + truncated_vos.get(i).getPoint().getY());
                line_list.getPoints().add(p3);

                Point p4 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
                p4.setX(p.getX() + 3 * truncated_vos.get(i).getRightLegDir().getX());
                p4.setY(p.getY() + 3 * truncated_vos.get(i).getRightLegDir().getY());
                line_list.getPoints().add(p4);

            }
        } else {
            for (int i = 0; i < truncated_vos.size(); i++) {
                Point p = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
                p.setX(pos.getPos().getX() + truncated_vos.get(i).getTruncLeft().getX());
                p.setY(pos.getPos().getY() + truncated_vos.get(i).getTruncLeft().getY());
                line_list.getPoints().add(p);

                Point p1 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
                p1.setX(p.getX() + 3 * truncated_vos.get(i).getLeftLegDir().getX());
                p1.setY(p.getY() + 3 * truncated_vos.get(i).getLeftLegDir().getY());
                line_list.getPoints().add(p1);

                Point p2 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
                p2.setX(pos.getPos().getX() + truncated_vos.get(i).getTruncRight().getX());
                p2.setY(pos.getPos().getY() + truncated_vos.get(i).getTruncRight().getY());
                line_list.getPoints().add(p2);

                Point p3 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
                p3.setX(p.getX() + 3 * truncated_vos.get(i).getRightLegDir().getX());
                p3.setY(p.getY() + 3 * truncated_vos.get(i).getRightLegDir().getY());
                line_list.getPoints().add(p3);

                Point p4 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
                p4.setX(pos.getPos().getX() + truncated_vos.get(i).getTruncLeft().getX());
                p4.setY(pos.getPos().getY() + truncated_vos.get(i).getTruncLeft().getY());
                line_list.getPoints().add(p4);

                Point p5 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
                p5.setX(pos.getPos().getX() + truncated_vos.get(i).getTruncRight().getX());
                p5.setY(pos.getPos().getY() + truncated_vos.get(i).getTruncRight().getY());
                line_list.getPoints().add(p5);
            }
        }
        vo_pub.publish(line_list);
    }

    public void publishPlan(List<PoseStamped> path,String name_space, Publisher path_pub) {
        MarkerArray pathPoint_array = MsgPublisherNode.getTopicMessageFactory().newFromType(MarkerArray._TYPE);
        if (!(path.size()==0)) {
            Header hd = path.get(0).getHeader();
            for (int i = 0; i < path.size(); i++) {

                Marker pathPoint = MsgPublisherNode.getTopicMessageFactory().newFromType(Marker._TYPE);

                pathPoint.setHeader(hd);
                pathPoint.setNs(name_space);
                pathPoint.setAction(Marker.ADD);
                pathPoint.setType(Marker.ARROW);
                pathPoint.getScale().setX(0.05);
                pathPoint.getScale().setY(0.03);
                pathPoint.getScale().setZ(0.03);
                pathPoint.getColor().setG((float) 0.5);
                pathPoint.getColor().setA(1);
                pathPoint.setId(i);
                pathPoint.setPose(path.get(i).getPose());
                pathPoint_array.getMarkers().add(pathPoint);
            }
        }
        path_pub.publish(pathPoint_array);
    }


    //velocity samples
    public void publishPoints(Position pos, List<VelocitySample> points, String base_frame, String name_space, Publisher samples_pub) {
        MarkerArray point_array = MsgPublisherNode.getTopicMessageFactory().newFromType(MarkerArray._TYPE);

        for (int i = 0; i < points.size(); i++) {

            Marker sphere = MsgPublisherNode.getTopicMessageFactory().newFromType(Marker._TYPE);

            sphere.getHeader().setFrameId(base_frame);
            sphere.getHeader().setStamp(MsgPublisherNode.getCurrentTime());
            sphere.setNs(name_space);
            sphere.setAction(Marker.ADD);
            sphere.getPose().getOrientation().setW(1);
            sphere.setType(Marker.SPHERE);
            sphere.getScale().setX(0.05);
            sphere.getScale().setY(0.05);
            sphere.getScale().setZ(0.05);
            sphere.getColor().setB((float) 0.5);
            sphere.getColor().setA(1);
            sphere.setId(i);
            Point p = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
            p.setX(pos.getPos().getX() + points.get(i).getVelocity().getX());
            p.setY(pos.getPos().getY() + points.get(i).getVelocity().getY());
            p.setZ(0.5);
            sphere.getPoints().add(p);
            sphere.getPose().getPosition().setX(p.getX());
            sphere.getPose().getPosition().setY(p.getY());
            sphere.getPose().getPosition().setZ(p.getZ());
            point_array.getMarkers().add(sphere);
        }

        for (int i = point_array.getMarkers().size(); i < MAX_POINTS_; i++) {
            Marker sphere_list = MsgPublisherNode.getTopicMessageFactory().newFromType(Marker._TYPE);
            sphere_list.getHeader().setFrameId(base_frame);
            sphere_list.getHeader().setStamp(MsgPublisherNode.getCurrentTime());
            sphere_list.setNs(name_space);
            sphere_list.setAction(Marker.DELETE);
            sphere_list.setId(i);
            Point p = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
            sphere_list.getPoints().add(p);
            sphere_list.getPose().getPosition().setX(p.getX());
            sphere_list.getPose().getPosition().setY(p.getY());
            sphere_list.getPose().getPosition().setZ(p.getZ());

            point_array.getMarkers().add(sphere_list);
        }
        samples_pub.publish(point_array);
    }

    // currently use clear path method additional constraints (orca_lines) include non holonomic robot constraints, acceleration constraints.
    public void publishOrcaLines(List<Line> orca_lines, Position position, String frame, String name_space, Publisher line_pub) {
        Marker line_list = MsgPublisherNode.getTopicMessageFactory().newFromType(Marker._TYPE);
        line_list.getHeader().setFrameId(frame);
        line_list.getHeader().setStamp(MsgPublisherNode.getCurrentTime());
        line_list.setNs(name_space);
        line_list.setAction(Marker.ADD);
        line_list.getPose().getOrientation().setW(1.0);
        line_list.setType(Marker.LINE_LIST);
        line_list.getScale().setX(0.01);
        line_list.getColor().setR((float)0.5);
        line_list.getColor().setA(1);
        line_list.setId(1);

        for (int i = 0; i < orca_lines.size(); i++) {
            Point p = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
            // orca lines are in robot frame so need to add position
            p.setX(position.getPos().getX() + orca_lines.get(i).getPoint().getX() - orca_lines.get(i).getDir().getX());
            p.setY(position.getPos().getY() + orca_lines.get(i).getPoint().getY() - orca_lines.get(i).getDir().getY());

            line_list.getPoints().add(p);
            Point p1 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
            p1.setX(p.getX() + 3 * orca_lines.get(i).getDir().getX());
            p1.setY(p.getY() + 3 * orca_lines.get(i).getDir().getY());
            line_list.getPoints().add(p1);
        }

        line_pub.publish(line_list);

    }



    public void publishObstacleLines(List<Obstacle> obstacles_lines, String frame, String name_space, Publisher line_pub) {
        Marker line_list = MsgPublisherNode.getTopicMessageFactory().newFromType(Marker._TYPE);
        line_list.getHeader().setFrameId(frame);
        line_list.getHeader().setStamp(MsgPublisherNode.getCurrentTime());
        line_list.setNs(name_space);
        line_list.setAction(Marker.ADD);
        line_list.getPose().getOrientation().setW(1);
        line_list.setType(Marker.LINE_LIST);
        line_list.getScale().setX(0.015);
        line_list.getColor().setR(1);
        line_list.getColor().setA(1);
        line_list.setId(1);
        for (int i = 0; i < obstacles_lines.size(); i++) {
            Point p = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
            Point p1 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);

            if (obstacles_lines.get(i).getBegin().equals(obstacles_lines.get(i).getEnd())) {
                continue;
            }
            p.setX(obstacles_lines.get(i).getBegin().getX());
            p.setY(obstacles_lines.get(i).getBegin().getY());
            line_list.getPoints().add(p);

            p1.setX(obstacles_lines.get(i).getEnd().getX());
            p1.setY(obstacles_lines.get(i).getEnd().getY());
            line_list.getPoints().add(p1);

        }
        line_pub.publish(line_list);

    }


    public void publishMePosition(ROSAgent me, String base_frame, String name_space, Publisher me_pub) {
        MarkerArray sphere_list = MsgPublisherNode.getTopicMessageFactory().newFromType(MarkerArray._TYPE);
        sphere_list.getMarkers().clear();
        fillMarkerWithROSAgent(sphere_list, me, base_frame, name_space);
        me_pub.publish(sphere_list);
    }


    public void publishNeighborPositions(List<ROSAgent> neighbors, String base_frame, String name_space, Publisher neighbors_pub) {
        MarkerArray sphere_list = MsgPublisherNode.getTopicMessageFactory().newFromType(MarkerArray._TYPE);
        sphere_list.getMarkers().clear();

        for (int i = 0; i < neighbors.size(); i++) {
            fillMarkerWithROSAgent(sphere_list, neighbors.get(i), base_frame, name_space);
        }
        neighbors_pub.publish(sphere_list);
    }

    private void fillMarkerWithROSAgent(MarkerArray markers, ROSAgent agent, String base_frame, String name_space) {
        int id = markers.getMarkers().size();
                //resize
        for (int i = 0; i < 2; i++) {
            Marker tmpMarker = MsgPublisherNode.getTopicMessageFactory().newFromType(Marker._TYPE);
            markers.getMarkers().add(tmpMarker);
        }

        markers.getMarkers().get(id).getHeader().setFrameId(base_frame);
        markers.getMarkers().get(id).getHeader().setStamp(MsgPublisherNode.getCurrentTime());
        markers.getMarkers().get(id).setNs(name_space);
        markers.getMarkers().get(id).setAction(Marker.ADD);
        markers.getMarkers().get(id).getPose().getOrientation().setW(1);
        markers.getMarkers().get(id).setType(Marker.CYLINDER);
        markers.getMarkers().get(id).getScale().setX(2 * agent.getRadius());
        markers.getMarkers().get(id).getScale().setY(2 * agent.getRadius());
        markers.getMarkers().get(id).getScale().setZ(0.5);
        markers.getMarkers().get(id).getColor().setR(1);
        markers.getMarkers().get(id).getColor().setA(1);
        markers.getMarkers().get(id).setId(id);

        double yaw, x_dif, y_dif, th_dif, time_dif;
        time_dif = MsgPublisherNode.getCurrentTime().toSeconds() - agent.getLastSeen().toSeconds();
        yaw = LPutils.getYaw(agent.getBaseOdom().getPose().getPose().getOrientation());
        //ROS_ERROR("time_dif %f", time_dif);
        //time_dif = 0.0;
        //forward projection?
        th_dif = time_dif * agent.getBaseOdom().getTwist().getTwist().getAngular().getZ();
        if (agent.getIsHolo()) {
            x_dif = time_dif * agent.getBaseOdom().getTwist().getTwist().getLinear().getX();
            y_dif = time_dif * agent.getBaseOdom().getTwist().getTwist().getLinear().getY();
        } else {
            x_dif = time_dif * agent.getBaseOdom().getTwist().getTwist().getLinear().getX() * Math.cos(yaw + th_dif / 2.0);
            y_dif = time_dif * agent.getBaseOdom().getTwist().getTwist().getLinear().getX() * Math.sin(yaw + th_dif / 2.0);
        }

        double x=agent.getBaseOdom().getPose().getPose().getPosition().getX() + x_dif;
        double y=agent.getBaseOdom().getPose().getPose().getPosition().getY() + y_dif;

        markers.getMarkers().get(id).getPose().getPosition().setX(x);
        markers.getMarkers().get(id).getPose().getPosition().setY(y);
        //central point
        markers.getMarkers().get(id).getPose().getPosition().setZ(0.25);

        id = id + 1;
        markers.getMarkers().get(id).getHeader().setFrameId(base_frame);
        markers.getMarkers().get(id).getHeader().setStamp(MsgPublisherNode.getCurrentTime());
        markers.getMarkers().get(id).setNs(name_space);
        markers.getMarkers().get(id).setAction(Marker.ADD);
        markers.getMarkers().get(id).getPose().getOrientation().setW(1);//tf::createQuaternionMsgFromYaw(yaw+th_dif);
        markers.getMarkers().get(id).setType(Marker.ARROW);
        markers.getMarkers().get(id).getScale().setX(0.05);
        markers.getMarkers().get(id).getScale().setY(0.1);
        markers.getMarkers().get(id).getColor().setG(1);
        markers.getMarkers().get(id).getColor().setA(1);
        markers.getMarkers().get(id).setId(id);

        Point p1 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
        p1.setX(x);
        p1.setY(y);
        p1.setZ(0.25);
        markers.getMarkers().get(id).getPoints().add(p1);

        Point p2 = MsgPublisherNode.getTopicMessageFactory().newFromType(Point._TYPE);
        p2.setX(x + agent.getRadius() * 2.0 * Math.cos(yaw + th_dif));
        p2.setY(y + agent.getRadius() * 2.0 * Math.sin(yaw + th_dif));
        p2.setZ(0.25);
        markers.getMarkers().get(id).getPoints().add(p2);
    }
}



