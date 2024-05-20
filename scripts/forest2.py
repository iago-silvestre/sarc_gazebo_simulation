#!/usr/bin/env python

import rospy
import rospkg
import random
import math
import os
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Duration

def spawn_tree(model_name, model_pose, model_path):
    """
    Function to spawn a tree model at a given pose.
    """
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        with open(model_path, "r") as f:
            model_sdf = f.read()
        spawn_model(model_name, model_sdf, "", model_pose, "world")
        rospy.loginfo("Spawned model {}".format(model_name))
    except rospy.ServiceException as e:
        rospy.logerr("Spawn service call failed: {}".format(e))

def get_package_path(package_name):
    """
    Function to get the path of a ROS package.
    """
    return rospy.get_param('/{}/path'.format(package_name))

def main():
    rospy.init_node('spawn_forest')
    rospack = rospkg.RosPack()
    # Define forest parameters
    forest_size = 10  # Size of the forest (number of trees per side)
    tree_spacing = 2  # Spacing between trees
    colors = ['tree_red', 'tree_yellow', 'tree_green']
    min_distance = 20  # Minimum distance from the center
    max_distance = 50  # Maximum distance from the center
    
    # Randomly generate the distance from the center
    distance_from_center = random.uniform(min_distance, max_distance)

    # Ensure distance from center is within the desired range
    distance_from_center = max(min_distance, min(max_distance, distance_from_center))

    # Define spawn offset based on random distance from the center
    offset = distance_from_center - (forest_size - 1) * tree_spacing / 2.0

    # Counter for naming trees
    tree_count = 1

    # Spawn trees in a square grid for each color
    for color in colors:
        # Adjust the offset for each color
        color_offset = offset * (colors.index(color) + 1)
        for i in range(forest_size):
            for j in range(forest_size):
                # Calculate spawn position for each tree
                x = i * tree_spacing - color_offset
                y = j * tree_spacing - color_offset
                # Determine color based on distance from the center
                distance = math.sqrt(x**2 + y**2)
                # Spawn the tree with the determined color
                if distance < tree_spacing:
                    print("x: "+ str(x) + ", y: "+ str(y))
                    model_name = '{}_{}'.format(color, tree_count)  # Unique name for each tree
                    model_path = rospack.get_path('sarc_gazebo_simulation') + "/models/"+color +"/model.sdf"
                    pose = Pose(position=Point(x, y, 0), orientation=Quaternion(0, 0, 0, 1))
                    spawn_tree(model_name, pose, model_path)
                    rospy.sleep(0.1)  # Add a small delay between spawning trees
                    tree_count += 1  # Increment tree count

    rospy.spin()

if __name__ == '__main__':
    main()
