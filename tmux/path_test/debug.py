#!/usr/bin/env python

import rospy
from mrs_msgs.msg import Path, Reference
from geometry_msgs.msg import PoseStamped

class WaypointTracker:
    def __init__(self):
        self.threshold = 0.1  # Threshold distance to consider passing a waypoint
        self.last_waypoint_index = 0
        self.waypoints = []

        # Subscribe to the /uav1/trajectory_generation/path topic
        rospy.Subscriber("/uav1/trajectory_generation/path", Path, self.path_callback)

        # Subscribe to the drone's pose
        rospy.Subscriber("/uav1/pose", PoseStamped, self.pose_callback)

        # Publisher for the last waypoint index
        self.last_waypoint_pub = rospy.Publisher("/last_waypoint_index", PoseStamped, queue_size=10)

    # Callback function to handle incoming Path messages
    def path_callback(self, data):
        self.waypoints = [(point.position.x, point.position.y) for point in data.points]

    # Callback function to handle incoming Pose messages
    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Check if waypoints list is empty
        if not self.waypoints:
            return
        
        # Calculate distance to next waypoint
        next_waypoint_index = (self.last_waypoint_index + 1) % len(self.waypoints)
        next_waypoint_x, next_waypoint_y = self.waypoints[next_waypoint_index]
        distance = ((x - next_waypoint_x) ** 2 + (y - next_waypoint_y) ** 2) ** 0.5
        
        # Check if drone has passed the next waypoint
        if distance < self.threshold:
            # Update last waypoint index
            self.last_waypoint_index = next_waypoint_index
            
            # Publish last waypoint index
            last_waypoint_msg = PoseStamped()
            last_waypoint_msg.pose.position.x = next_waypoint_x
            last_waypoint_msg.pose.position.y = next_waypoint_y
            self.last_waypoint_pub.publish(last_waypoint_msg)

def main():
    # Initialize the ROS node
    rospy.init_node('waypoint_tracker', anonymous=True)

    # Create an instance of the WaypointTracker class
    waypoint_tracker = WaypointTracker()

    # Spin to keep the node running and process incoming messages
    rospy.spin()

if __name__ == '__main__':
    main()
