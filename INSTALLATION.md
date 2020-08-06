Throughout these instructions, if you need to build something use
```
catkin build
```
while in your catkin workspace

### 1. Install ROS and Gazebo for use with PX4
[This](https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html#rosgazebo) provides a script that should install everything quite easily. \
Run `sudo rosdep init` and `rosdep update` in ~/catkin_ws after the script finishes.
Everything should work successfully up until the packages start to be built in ~/catkin_ws. So see below for the solutions to some errors you may encounter.
#### Possible Errors
I. `ImportError: No module named future`\
Solution: `pip install future`\
II. `Could not find a package configuration file provided by "geographic_msgs"`\
Solution: `sudo apt-get install ros-melodic-geographic-msgs`\
III. `Could NOT find GeographicLib (missing: GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS)`\
Solution: `sudo apt-get install libgeographic-dev`
### 2. Install a bunch of other packages
Follow the instructions [here](https://dev.px4.io/master/en/simulation/gazebo_octomap.html). Make sure to install all packages with the `ros-melodic` prefix, **NOT** `ros-indigo` or another ROS distribution.
#### RotorS
There are a lot of commands, so if you're struggling, use the brief list of commands below. If you encounter more errors than what I have listed, it will be worth checking the link above.
```
sudo apt-get install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox`
cd ~/catkin_ws/src
wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
wstool merge rotors_hil.rosinstall
wstool update
```
After running `wstool update`, it will ask if you want to (d)elete and replace, (a)bort, (b)ackup and replace, (s)kip. Type `s`
```
catkin build
source ~/catkin_ws/devel/setup.bash
```
#### rviz, CGAL
The base rviz should be installed already, but the visual tools may not be.
```
sudo apt-get install ros-melodic-rviz ros-melodic-rviz-visual-tools libcgal-dev
```
#### Octomap world, Catkin Simple eigen_catkin, gflags_catkin, glog_catkin, minkindr, eigen_checks, catkin_boost_python_buildtool, numpy_eigen, minkindr_conversions, octomap_rviz_plugins
```
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/volumetric_mapping.git
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/eigen_catkin.git
git clone https://github.com/ethz-asl/gflags_catkin.git
git clone https://github.com/ethz-asl/minkindr.git
git clone https://github.com/ethz-asl/eigen_checks.git
git clone https://github.com/ethz-asl/catkin_boost_python_buildtool.git
git clone https://github.com/ethz-asl/numpy_eigen.git
git clone https://github.com/ethz-asl/minkindr_ros.git
git clone https://github.com/OctoMap/octomap_rviz_plugins.git
```
#### Possible Errors
I.
```
[Err] [REST.cc:205] Error in REST request

libcurl: (51) SSL: no alternative certificate subject name matches target host name 'api.ignitionfuel
```
Solution: In your favorite text editor, open `~/.ignition/fuel/config.yaml` and replace `api.ignitionfuel.org` with `fuel.ignitionrobotics.org`
### 3. Clone the author’s code into the src directory of your catkin workspace (likely located at ~/catkin_ws/src/)
There are 3 repositories that may be needed.\
https://github.com/kucars/sspp \
https://github.com/kucars/asscpp \
https://github.com/kucars/aircraft_inspection \
There are a lot of mysteries with these 3 repositories, so I’ll describe what they are and what I’ve been able to deduce.

The short answer is this: Get the 1st and 2nd, and if for some reason you get errors related to a package called “component_test”, download the 3rd.

The long answer, and my journey through these repositories is this:\
First, the main repository listed in the paper (number 1 in the list) has all the code related to the actual coverage path planning deleted. You can go through the commit history to find the files yourself, and you will notice a couple of things. First, you will see that it used to require a package called “component_test”. I found this package in the 3rd repo listed.\
But I don’t believe that restoring these deleted files and installing the component test package is necessary, as I also discovered the 2nd repository. In the folder called “cscpp”, I’ve found all the files that were deleted in the 1st, and the lines that require the component_test package have been commented out. Good news!\
I believe that after the paper was published, the authors went through and refactored the code so that repository 1 now only holds the framework for their path planning algorithms, and repository 2 holds the actual algorithm specific to the paper. So you should be fine just downloading the 1st and 2nd repository.
### 4. Edit some of the CMakefiles and code
Around line 60 of `sspp/CMakeLists.txt`, change
```
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
   CATKIN_DEPENDS roscpp pcl_ros octomap_world
)
```
To
```
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} SSPathPlanner
   CATKIN_DEPENDS roscpp pcl_ros octomap_world
)
```
Around line 51 of `cscpp/CMakeLists.txt` change
```
link_directories(${PCL_LIBRARY_DIRS})
```
To
```
link_directories(${PCL_LIBRARY_DIRS} ${sspp_LIBRARY_DIRS})
```
Around line 38 of `sspp/src/node.cpp` change
```
parent = next = prev = NULL;
cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
```
To
```
parent = next = prev = NULL;
octree = NULL;
cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
```
Around line 45 of `sspp/src/node.cpp` change
```
parent = n->parent;
next   = n->next;
prev   = n->prev;
depth  = n->depth;
```
To
```
parent = n->parent;
next   = n->next;
prev   = n->prev;
depth  = n->depth;
octree = n->octree;
```
In `cscpp/launch/coverage_heuristic_test.launch` change
```
<node pkg="cscpp" type="coverage_heuristic_test" name="coverage_heuristic_test"/>
```
To
```
<node pkg="cscpp" type="coverage_heuristic_test" name="coverage_heuristic_test" output="screen" />
```