# 1. Package overview
* `mir_actions`: Action definitions for the MiR robot
* `mir_description`: URDF description of the MiR robot
* `mir_dwb_critics`: Plugins for the dwb_local_planner used in Gazebo
* `mir_driver`: A reverse ROS bridge for the MiR robot
* `mir_gazebo`: Simulation specific launch and configuration files for the MiR robot
* `mir_msgs`: Message definitions for the MiR robot
* `mir_navigation`: move_base launch and configuration files


# 2. Installation
The instructions below use the ROS distro `kinetic` as an example.

### Preliminaries
If you haven't already installed ROS on your PC, you need to add the ROS apt
repository. This step is necessary for either binary or source install.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
```

### Binary install
For a binary install, it suffices to run this command:
```bash
sudo apt install ros-kinetic-mir-robot
sudo apt install ros-kinetic-navigation
```
See the tables at the end of this README for a list of ROS distros for which
binary packages are available.

### Source install
For a source install, run the commands below instead of the command from the
"binary install" section.
```bash
cd ~/catkin_ws/src/
```

### Clone mir_robot into the catkin workspace
```bash
git clone https://github.com/Jryl/MIR200_Sim_Demo.git
```
### Install all dependencies
```bash
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths ./ -i -y --rosdistro kinetic
```

### Build all packages in the catkin workspace
```bash
source /opt/ros/kinetic/setup.bash
catkin_init_workspace
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo
```


# 3. Start Gazebo demo (existing map)
```bash
roslaunch mir_navigation mir_start.launch
```

Now, you can use the "2D Nav Goal" tool in RViz to set a navigation goal for move_base.

# 4. Gazebo demo (mapping)
```bash
roslaunch mir_navigation mir_mapping.launch
```

# 5. Teleoperate the robot with keyboard (optional)
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```


#####################################################################################################
# Install WS to use with object detection and D435
#####################################################################################################
# INSTALL


├── librealsense
└── visp-ws
    ├── visp
    ├── visp-build
    ├── build
    ├── devel
    └── src
        ├── CMakeLists.txt
        ├── realsense_gazebo_plugin
        ├ 	├── package.xml
        ├ 	├── CMakeLists.txt
        ├	└── ...
        ├── realsense-ros
        ├ 	├── realsense2_camera
        ├	├	└── ...
        ├	└── realsense2_description
        ├		└── ...
        └── Mir200_withD435
         	├── mir_navigation
         	├── ...
        	└── mir_vision
         		├── CMakeLists.txt
         		├── package.xml
       	  		└── src
      	 		     ├── tutorial-viewer.cpp
      	 		     └── tutorial-mb-generic-tracker-rgbd-realsense.cpp

realsense-ros Version needs to fit the librealsense version!


# For "realsense-ros" ddynamic reconfigure is needed
sudo apt install ros-melodic-ddynamic-reconfigure

# The realsense-ros-2.2.20 needs "librealsense" to be installed.
# For this installation there are two options.
# Install from Debian Package 
# (e.g. https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages )
# or manual installation (my case)
# https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
# However pay attenntion on versions!
# librealsense2 and RosWrapper need to fit together -> fitting versions under latest releses


# The realsense-ros wrapper can be downloaded from 
# https://github.com/IntelRealSense/realsense-ros/releases
# There you will find an overview of the current versions and releases
# I used the ROS Wrapper 2.0 for Intel® RealSense™ Devices (build 2.2.20) (Hash fbbb1a7)
# which is compatible with my LibRealSense v2.40.0
# Under Assets download the zip and unzip it
# Save the folder in your catkin_ws/src



# To simulate Realsense Cam in Gazebo, the Package "realsense_gazebo_plugin" is needed.
# Further information about how to include the D435 Plugin with MiR 
# can be found in the Readme in Mir200_Sim_withD435 package/ see below

# Intel RealSense Gazebo ROS plugin
This package is a Gazebo ROS plugin for the Intel D435 realsense camera.
 
## Acknowledgement
This is a continuation of work done by [SyrianSpock](https://github.com/SyrianSpock) for a Gazebo ROS plugin with RS200 camera.
This package also includes the work developed by Intel Corporation with the ROS model fo the [D435](https://github.com/intel-ros/realsense) camera.

### Install Plugin (in MiR workspace)
1. Download this package from https://github.com/pal-robotics/realsense_gazebo_plugin, for example with:
	git clone -b melodic-devel https://github.com/pal-robotics/realsense_gazebo_plugin.git
2. Save it into your catkin_ws/src folder
3. Compile with catkin_make
-> This will generate a shared library called librealsense_gazebo_plugin.so

# The next steps are already done, if you clone HartmannSa/MiR200_Sim_withD435 Repo. 
# If you clone the rosmatch/MiR200_Sim repo they need do be done! 
4. Download the two xacro files from 
	https://github.com/pal-robotics-forks/realsense/tree/upstream/realsense2_description/urdf 
	_d435.gazebo.xacro 
	_d435.urdf.xacro
AND download the mesh from 
	https://github.com/pal-robotics-forks/realsense/tree/upstream/realsense2_description/meshes
	d435.dae
5. Save two xacro-files in the package inside the urdf folder, where you want your realsense to be simulated (e.g. ~/catkin_ws/src/MiR200/mir_description/urdf/include ).
The dae-file can be saved under mir_description/meshes/visual/.
Some Changes in the downloaded Files are made, so that they are loaded with the Mir Model AND that two cameras can be spawned (and dont publish on the same topic)
6. In the file _d435.urdf.xacro 
	change the packagename "realsense2_description" in line 13 
	<xacro:include filename="$(find realsense2_description)/urdf/urdf _d435.gazebo.xacro "> 
	to the correct one (e.g mir_description) 
	<xacro:include filename="$(find mir_description)/urdf/include/urdf_d435.gazebo.xacro "> 

	Change the Macro definition (line 15), so that there has to be a name to be defined 
	(necessary to distinguish between severalcameras)
	Before:
	After:
  	<xacro:macro name="sensor_d435" params="topics_ns:=camera name parent *origin">

	Correct the path to the desired mesh (line 62):
	<mesh filename="package://mir_description/meshes/visual/d435.dae" />

	Change the topic_ns param in line 142 (only then the cameras publsih to different topics):
   	 <xacro:gazebo_d435 camera_name="${name}" reference_link="${name}_link" topics_ns="${name}" 
	...

7. In the urdf file where you want to use the simulated realsense (e.g. mir_200_v1.urdf.xacro),
	add the following code, (while again replacing packagename with the name of the package
	(e.g. mir_description) you are using and "baselink" whith the link you want the 
	realsense camera to be joint on (e.g. camera_link) )
	<xacro:include filename="$(find mir_description)/urdf/include/_d435.urdf.xacro">
	<!-- Camera -->
    	<xacro:sensor_d435 name="camera_base" parent="${prefix}base_link">
      		<origin xyz="0.42 0 0.27" rpy="0 0.2 0"/>
    	</xacro:sensor_d435>
   	 <xacro:sensor_d435 name="camera_arm" parent="${prefix}arm_1_link">
      		<origin xyz="0.025 0 0.26" rpy="0 0.2 0"/>
    	</xacro:sensor_d435>

	Obviously, all old camera tags, etc has to be uncommented/ deleted.

8. in mir_200.gazebo.xacro the camera tag is not needed anymore and should be deleted.
9. Afterwards when you launch the file which uses this urdf model,
	 you will find the simulate realsense and you should see rostopics like 
	/camera_arm/color/image_raw




