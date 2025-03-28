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
from sensor_msgs.msg import Image
import threading
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class WaypointTrackerNode:
    def __init__(self, n):
        rospy.init_node('waypoint_tracker_node', anonymous=True)
        self.n_drones = n
        # Parameters
        self.threshold = 1.0
        self.battery_levels = [100.0 for _ in range(n)]
        #self.battery_levels[1] = 33.0 # Demonstrate low battery
        self.fireSize=4
        self.count=0
        self.auxcount=0
        self.waypoints = [[] for _ in range(n)]
        self.last_waypoint_indices = [0 for _ in range(n)]
        self.path_publishers = []
        self.fire_detection_publishers = []
        self.battery_publishers = []
        self.bridge = CvBridge()
        self.teste=0
              
        self.subscriber_del = rospy.Subscriber('/fightFire', Int32, self.del_callback)
        self.fire_size_pub = rospy.Publisher("fireSize", Int8, queue_size=1)

        # Delete model service
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        # Create a ROS timer to subtract 0.1 from battery variables every 2 seconds

        for i in range(n):
            rospy.Subscriber(f'/uav{i+1}/trajectory_generation/path', Path, self.create_path_callback(i))
            path_publisher = rospy.Publisher(f'uav{i+1}_lastWP', Int8, queue_size=1)
            self.path_publishers.append(path_publisher)
            rospy.Subscriber(f'/uav{i+1}/ground_truth', Odometry, self.create_odom_callback(i))
            rospy.Subscriber(f'/uav{i+1}/bluefox_optflow/image_raw', Image, self.create_image_callback(i))
            fire_detection_publisher = rospy.Publisher(f'/uav{i+1}/fire_detection', Int32, queue_size=1)
            self.fire_detection_publishers.append(fire_detection_publisher)
            battery_publisher = rospy.Publisher(f'battery_uav{i+1}', Float64, queue_size=1)
            self.battery_publishers.append(battery_publisher)
        rospy.Timer(rospy.Duration(1), self.update_batteries)

        rospy.Subscriber('/recharge_battery', Int8, self.recharge_battery_callback)

        #self.fire_size_pub.publish(self.fireSize)

    def update_batteries(self, event):
        # Subtract 0.1 from each battery variable and publish
        for i in range(self.n_drones):
            self.battery_levels[i] -= 0.1
            #print(self.battery_levels[i])
            self.battery_publishers[i].publish(self.battery_levels[i])   
            self.fire_size_pub.publish(self.fireSize)

    def recharge_battery_callback(self, msg):
        drone_index = msg.data - 1
        if 0 <= drone_index < self.n_drones:
            self.battery_levels[drone_index] = 100.0
            self.battery_publishers[drone_index].publish(100.0)
            rospy.loginfo("Recharged battery of UAV%d to 100", drone_index + 1)
        else:
            rospy.logwarn("Invalid drone index: %d", drone_index + 1)
         
    def create_path_callback(self, drone_index):
        def path_callback(msg):
            self.waypoints[drone_index] = [(point.position.x, point.position.y) for point in msg.points]
            #for i, (x, y) in enumerate(self.waypoints[drone_index]):
                #rospy.loginfo("Drone %d Waypoint %d: x=%.2f, y=%.2f", drone_index + 1, i, x, y)
            # Initialize last waypoint index
            self.last_waypoint_indices[drone_index] = 0
            self.path_publishers[drone_index].publish(0)
        return path_callback
    
    """def create_odom_callback(self, drone_index):   #This method only checks the next waypoint, some issues can occur due to collision avoidance
        def odom_callback(msg):
            # Extract drone position
            if not self.waypoints[drone_index]:
                #rospy.logwarn("No waypoints received yet for drone %d. Skipping odometry callback.", drone_index + 1)
                return
            #print(msg)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # Calculate distance to next waypoint
            next_waypoint_index = (self.last_waypoint_indices[drone_index]) % len(self.waypoints[drone_index])
            next_waypoint_x, next_waypoint_y = self.waypoints[drone_index][next_waypoint_index]
            distance = ((x - next_waypoint_x) ** 2 + (y - next_waypoint_y) ** 2) ** 0.5
            #print("distance = %.2f"%distance)
            # Check if drone has passed the next waypoint
            if distance < self.threshold:
                # Update last waypoint index
                self.last_waypoint_indices[drone_index] = next_waypoint_index +1
                
                # Publish last waypoint index
                self.path_publishers[drone_index].publish(next_waypoint_index + 1)
        return odom_callback"""

    def create_odom_callback(self, drone_index):
        def odom_callback(msg):
            # Extract drone position
            if not self.waypoints[drone_index]:
                return
            
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # Check all waypoints to find the closest one within the threshold
            for i, (waypoint_x, waypoint_y) in enumerate(self.waypoints[drone_index]):
                distance = ((x - waypoint_x) ** 2 + (y - waypoint_y) ** 2) ** 0.5
                if distance < self.threshold:
                    # Update last waypoint index
                    self.last_waypoint_indices[drone_index] = i + 1  # Waypoint indices are 1-based for publishing
                    # Publish last waypoint index
                    self.path_publishers[drone_index].publish(i + 1)
                    break
        return odom_callback

    def create_image_callback(self, drone_index):
        def image_callback(msg):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
                return

            # Convert the image to HSV color space
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            # Define the red color range
            lower_red = np.array([150, 0, 0])
            upper_red = np.array([255, 100, 100])
            # Create a mask to extract only the red pixels
            red_mask = cv2.inRange(cv_image, lower_red, upper_red)
            # Count the number of red pixels
            red_pixel_count = np.sum(red_mask == 255)
            # Publish the number of red pixels
            self.fire_detection_publishers[drone_index].publish(red_pixel_count)
            # Add a delay of 0.25 seconds
            #rospy.sleep(0.25)
        return image_callback

    def del_callback(self, msg):
        if msg.data != 0:
            model_name = "tree_red_"+ str(self.count)  # Change this to the name of the model you want to delete
            rospy.loginfo("Deleting model: %s", model_name)
            self.auxcount=self.auxcount+1
            probability = random.random()
            if self.auxcount != -2:
                #if probability <= 1.80:
                #if self.teste !=0:
                if probability <= 0.75:
                    try:
                        self.count=self.count+1
                        self.fireSize=self.fireSize-1
                        response = self.delete_model(model_name)
                        rospy.loginfo("Model deletion response: %s", response.status_message)
                        self.fire_size_pub.publish(self.fireSize)
                    except rospy.ServiceException as e:
                        rospy.logerr("Service call failed: %s", e)
            self.teste=self.teste+1
        
    


if __name__ == '__main__':
    n_drones = 6
    try:
        waypoint_tracker_node = WaypointTrackerNode(n_drones)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass