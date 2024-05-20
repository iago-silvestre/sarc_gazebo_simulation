#!/usr/bin/env python

import rospy
import rospkg
import random
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32,Float64

class ModelSpawner:
    def __init__(self):
        rospy.init_node('model_spawner')

        # Wait for Gazebo services to become available
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')

        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.subscriber = rospy.Subscriber('/fightFire', Int32, self.callback)
        # Get the path to the SDF file
        rospack = rospkg.RosPack()
        self.sdf_green = rospack.get_path('sarc_gazebo_simulation') + "/models/tree_green/model.sdf"
        self.count=0
        #self.createForest()

    def callback(self, msg):
        if msg.data != 0:
            model_name = "tree_red_"+ str(self.count)  # Change this to the name of the model you want to delete
            rospy.loginfo("Deleting model: %s", model_name)
            self.count=self.count+1
            probability = random.random()
            if probability <= 0.3:
                try:
                    response = self.delete_model(model_name)
                    rospy.loginfo("Model deletion response: %s", response.status_message)
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s", e)
                
    def spawn_model_random_pose(self):
        model_name = "tree" 

        # Randomly generate position and orientation
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0.5  # Assuming ground plane is at z=0
        # Assuming orientation in roll, pitch, yaw order
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0

        # Spawn model
        with open(self.sdf_green, "r") as f:
            sdf = f.read()
        try:
            self.spawn_model(model_name, sdf, "robot_description", pose, "world")
            rospy.loginfo("Spawned model: %s", model_name)
        except rospy.ServiceException as e:
            rospy.logerr("Spawn service call failed: %s", e)

        # Schedule deletion after some time
        rospy.Timer(rospy.Duration(10), lambda event: self.delete_model(model_name))
    
    

if __name__ == '__main__':
    try:
        spawner = ModelSpawner()
        #spawner.spawn_model_random_pose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

