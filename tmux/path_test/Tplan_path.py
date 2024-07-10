#!/usr/bin/python3

import rospy
import random
import os
import time

from mrs_msgs.msg import Reference, Path

class Goto:

    def __init__(self):
        rospy.init_node('goto', anonymous=True)
        rospy.loginfo('ROS node initialized')

        # Publisher for path
        self.path_pub = rospy.Publisher('/uav1/trajectory_generation/path', Path, queue_size=0)

        path_msg = Path()
        path_msg2= Path()

        path_msg.header.frame_id = ""
        path_msg.use_heading = True
        path_msg.fly_now = True
        path_msg.max_execution_time = 5.0
        path_msg.max_deviation_from_path = 0.0
        path_msg.dont_prepend_current_state = False
        path_msg.stop_at_waypoints = False; 

        path_msg2.header.frame_id = ""
        path_msg2.use_heading = True
        path_msg2.fly_now = True
        path_msg2.max_execution_time = 5.0
        path_msg2.max_deviation_from_path = 0.0
        path_msg2.dont_prepend_current_state = False
        path_msg2.stop_at_waypoints = False; 
        #path_msg2 = path_msg
        #path_msg2.header.frame_id = "2" 
        for i in range(0, 11):
            point = Reference()
            point.position.x = i * 5
            point.position.y = 0
            point.position.z = 7
            point.heading = -1.571
            path_msg.points.append(point)

        for k in range(0, 11):
            point = Reference()
            point.position.x = 50
            point.position.y = -(k * 5)
            point.position.z = 7
            point.heading = 0
            #point.heading = 1.571
            path_msg2.points.append(point)


        try:
            time.sleep(1)   
            path_msg.header.stamp = rospy.Time.now()
            rospy.loginfo('teste')
            self.path_pub.publish(path_msg)
            time.sleep(10)  
            path_msg2.header.stamp = rospy.Time.now()
            rospy.loginfo('teste2')
            self.path_pub.publish(path_msg2)

        except rospy.ROSException as e:
            rospy.logerr('[SweepingGenerator]: Exception occurred: {}'.format(e))

if __name__ == '__main__':
    try:
        goto = Goto()
        rospy.spin()  # keep the node alive
    except rospy.ROSInterruptException:
        pass