#!/usr/bin/python3

import rospy
import rosnode
import random
import os

from mrs_msgs.msg import Reference, Path

class planPath:

    def __init__(self):
        rospy.init_node("path1", anonymous=True)

        # Publisher for path
        self.path_pub = rospy.Publisher("/uav1/trajectory_generation/path", Path, queue_size=1)

        path_msg = Path()
        
        path_msg.header.frame_id = ""
        path_msg.use_heading = True
        path_msg.fly_now = True
        path_msg.max_execution_time = 5.0
        path_msg.max_deviation_from_path = 0.0
        path_msg.dont_prepend_current_state = False


        for i in range(0, 10):
            point = Reference()
            point.position.x = i * 10
            point.position.y = 0
            point.position.z = 7
            point.heading = -1.571
            path_msg.points.append(point)

        try:
            path_msg.header.stamp = rospy.Time.now()
            print(path_msg)
            self.path_pub.publish(path_msg)
        except:
            rospy.logerr('[SweepingGenerator]: path topic not available')
            pass

if __name__ == '__main__':
    try:
        path1 = planPath()
        rospy.spin()  # keep the node alive
    except rospy.ROSInterruptException:
        pass
