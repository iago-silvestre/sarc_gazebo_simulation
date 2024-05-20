# sarc_gazebo_simulation
### Dependencies

* Ubuntu 20.04
* Gazebo 11.10
* ROS Noetic
* MRS System 
* NOTE: MRS documentation https://ctu-mrs.github.io/

### Installing

* Since this package gives you just the environment, you can use it however you like, as long as you already have Ubuntu, ROS Noetic and Gazebo 11.10. 
  We strongly recommend that you use MRS System, which installs everything you need.
* Installing MRS:
* At the link https://github.com/ctu-mrs/mrs_uav_system , go to the Installation section and choose one of the installation options,
  any of them will work, we recommend that you use the local option, that you just run the script and everything is automatically downloaded.
* NOTE: Please only install MRS on a newly installed Ubuntu, it will be an easy way
* With all that done, simply clone this package into a Catkin workspace (if you don't know what that is, http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
  and build. (If you have MRS, we recommend that you clone into the "workspace" folder that is in "home")
* It is recommended to export the gazebo path to include this package models into the Gazebo simulation, which can be done by running the following in a terminal, or adding it to your ~/.bashrc file
 ```
export GAZEBO_MODEL_PATH=/home/oem/catkin_ws/src/sarc_gazebo_simulation/models/
```

### Executing simulation

* Choose a scenario inside the tmux directory and open a terminal inside it. Run the following to start the simulation, which is defined inside the session.yml file
```
./start.sh
```
* In another tab, you should run the auxiliary python node, that will colaborate with the Jason Agent to modify the Gazebo simulation
```
rosrun sarc_gazebo_simulation delete.py 
```
* Finally you should run the Jason Agents. That can be done by opening a terminal inside the TrajectEmasStd folder and running the following command
```
./gradlew
```
