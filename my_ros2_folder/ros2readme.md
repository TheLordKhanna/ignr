This readme describes the implementation of the ROS2 component of our project. While using ROS2, it is good practice to always apply the following -

1) packages should be inside your ROS2 workspace src folder
2) Always source ros after starting a new terminal - source /opt/ros/jazzy/setup.bash
3) Always rebuild your packages after you modify them. use - colcon build --packages-select (name) to build a specific package. always run source ./install/setup.bash after building
4) Always make sure you are in the right workspace root. cloning key packages like moveit into the wrong folder can lead to errors. To navigate through where your commands are passing, use cd .. to go to the parent folder and cd ./(folder name) to navigate to a folder

The igtl bridge package came pre-installed for our use, however, you can also clone ros2_igtl_bridge from the creators github repo. Importantly,on launching bridge.launch.py, we keep on receiving the error - libOpenIGTLink.so.3 not found. To solve this, use - export LD_LIBRARY_PATH=/usr/local/lib/igtl:$LD_LIBRARY_PATH


