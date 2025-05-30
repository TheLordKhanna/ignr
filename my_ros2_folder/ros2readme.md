This readme considers the implementation of the ROS2 component of our project. While using ROS2, it is good practice to always apply the following -

1) packages should be inside your ROS2 workspace src folder
2) Always source ros after starting a new terminal - source /opt/ros/jazzy/setup.bash
3) Always rebuild your packages after you modify them. use - colcon build --packages-select (name) to build a specific package. always run source ./install/setup.bash after building
4) Always make sure you are in the right workspace root. cloning key packages like moveit into the wrong folder can lead to errors. To navigate through where your commands are passing, use cd .. to go to the parent folder and cd ./(folder name) to navigate to a folder

The igtl bridge package came pre-installed for our use, however, you can also clone ros2_igtl_bridge from the creators github repo. Importantly,on launching bridge.launch.py, we keep on receiving the error - libOpenIGTLink.so.3 not found. To solve this, use - export LD_LIBRARY_PATH=/usr/local/lib/igtl:$LD_LIBRARY_PATH

If you choose to create a new moveit package using setup assistant but are using the same code in DVpack, please remember to change the names of the planning group, package, etc. in moveitcommanderfile.py. Additionally, if you choose to make a new py executable inside DVpack, please specify in setup.py or it will not show up. 

As such, if you ensure that these packages are implemented in the right workspace, everything is built and sourced regularly, then the main line of code to execute is ROS2 launch DVpack dvpack_bridge.launch.py. If you would only like to visualise the robot and perform random trajectory generation, you can use ROS2 launch the_ignr_moveit demo.launch.py. 

Common errors - 
1) libOpenIGTLink.so.3 not found. To solve this, use - export LD_LIBRARY_PATH=/usr/local/lib/igtl:$LD_LIBRARY_PATH
2) moveit: no controllers found - reinstall moveit, or use ros2 update and upgrade to make sure all libraries are present. if you run ldd to check the installed libraries, make sure that none of them are indicated as 'not found'
3) IGTL connection failed - make sure your slicer is an active server and that you have specified the correct port, client and IP
4) Trajectory failed - most likely an issue with the point being outside your workspace. Either modify the point or create a robot with larger workspace
