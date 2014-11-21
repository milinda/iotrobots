#!/usr/bin/env python
import rospy
from nav_msgs import OccupancyGrid
import costmap_2d

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('local_costmap_listener', anonymous=True)

    rospy.Subscriber("/robot2/move_base/local_costmap/costmap", OccupancyGrid, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()