#!/usr/bin/env python

import rospy
import rospkg
import random
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32,Float64
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from mrs_msgs.srv import PathSrv,PathSrvRequest
from mrs_msgs.msg import Reference,Path
import threading

class WaypointTrackerNode:
    def __init__(self):
        rospy.init_node('waypoint_tracker_node', anonymous=True)
        
        # Parameters
        self.threshold = 0.5
        
        # Initialize last waypoint index
        self.last_waypoint_index = -1
        
        # Subscribers
        rospy.Subscriber("uav1/ground_truth", Odometry, self.odometry_callback)
        
        # Publisher
        self.last_waypoint_pub = rospy.Publisher("uav1_lastWP", Int8, queue_size=1)
        
        self.fire_ext_pub = rospy.Publisher("fireExt", Int8, queue_size=1)

        # Rate for the odometry callback
        self.rate = rospy.Rate(1)  # Execute once per second
        self.waypoints = [(-5.0, -5.0), (5.0, -5.0), (5.0, 5.0), (-5.0, 5.0)]
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.subscriber_del = rospy.Subscriber('/fightFire', Int32, self.del_callback)
        self.subscriber_path1 = rospy.Subscriber('/uav1/trajectory_generation/path', Path, self.path1_callback)
        # Get the path to the SDF file
        self.count=0
        #self.createForest()

    def path1_callback(self, msg):
        self.waypoints = [(point.position.x, point.position.y) for point in msg.points]
        for i, (x, y) in enumerate(self.waypoints):
            rospy.loginfo("Waypoint %d: x=%.2f, y=%.2f", i, x, y)
        #global points_uav1
        #points_uav1 = msg.points
        #print("Received points list with length:", len(points_uav1))
        #print("Points list:")
        #for i, point in enumerate(points_uav1):
        #    print("Point", i, ":", point)


    def del_callback(self, msg):
        if msg.data != 0:
            model_name = "tree_red_"+ str(self.count)  # Change this to the name of the model you want to delete
            rospy.loginfo("Deleting model: %s", model_name)
            
            probability = random.random()
            if probability <= 0.7:
                try:
                    self.count=self.count+1
                    response = self.delete_model(model_name)
                    rospy.loginfo("Model deletion response: %s", response.status_message)
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s", e)
            if self.count==4:
                self.fire_ext_pub.publish(self.count)
            
        
    def odometry_callback(self, msg):
        # Extract drone position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Calculate distance to next waypoint
        next_waypoint_index = (self.last_waypoint_index + 1) % len(self.waypoints)
        next_waypoint_x, next_waypoint_y = self.waypoints[next_waypoint_index]
        distance = ((x - next_waypoint_x) ** 2 + (y - next_waypoint_y) ** 2) ** 0.5
        
        #print("distance = %.2f"%distance)
        # Check if drone has passed the next waypoint
        if distance < self.threshold:
            # Update last waypoint index
            self.last_waypoint_index = next_waypoint_index
            
            # Publish last waypoint index
            self.last_waypoint_pub.publish(self.last_waypoint_index+1)


if __name__ == '__main__':
    try:
        waypoint_tracker_node = WaypointTrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass