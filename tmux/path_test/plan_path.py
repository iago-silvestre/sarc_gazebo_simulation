#!/usr/bin/python3

import rospy
import rosnode
import random
import os
import time

from mrs_msgs.srv import PathSrv,PathSrvRequest
from mrs_msgs.msg import Reference, Path

class Goto:

    def __init__(self):

        rospy.init_node('goto', anonymous=True)

        rospy.loginfo('ros not initialized')

        publishers = []
        n_uavs = 1

        ## | --------------------- service clients -------------------- |

        self.sc_path = rospy.ServiceProxy('/uav1/trajectory_generation/path', PathSrv)

        path_msg = PathSrvRequest()
        path_msg2 = PathSrvRequest()

        path_msg.path.header.frame_id = ""
        

        path_msg.path.use_heading = True
        path_msg.path.fly_now = True

        path_msg.path.max_execution_time = 5.0
        path_msg.path.max_deviation_from_path = 0.0
        path_msg.path.dont_prepend_current_state = False
        path_msg2=path_msg

        for i in range(0, 10):

            point = Reference()

            point.position.x = i*10
            point.position.y = 0
            point.position.z = 7
            point.heading = -1.571

            path_msg.path.points.append(point)

        for k in range(0, 10):

            point = Reference()

            point.position.x = -k*10
            point.position.y = 0
            point.position.z = 7
            point.heading = 1.571

            path_msg2.path.points.append(point)
             
        try:
            path_msg.path.header.stamp = rospy.Time.now()
            print(path_msg)
            response = self.sc_path.call(path_msg)

            time.sleep(10)
            path_msg2.path.header.stamp = rospy.Time.now()
            print(path_msg2)
            response = self.sc_path.call(path_msg2)
        except:
            rospy.logerr('[SweepingGenerator]: path service not callable')
            pass

if __name__ == '__main__':
    try:
        goto = Goto()
    except rospy.ROSInterruptException:
        pass
