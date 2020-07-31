# SSPP-Complete
## About
This repository contatins all the information about how to get the code detailed in “Coverage Path Planning for Complex Structures Inspection Using Unmanned Aerial Vehicle (UAV)” up and running

## Installation
The [Installation page](https://gitlab.com/Bigwolfden/sspp-complete/-/wikis/Installation) of the wiki is a (hopefully) comprehensive guide on how to get the code for “Coverage Path Planning for Complex Structures Inspection Using Unmanned Aerial Vehicle (UAV)” up and running on Ubuntu 18.04.

## Utilities
There are two pieces of code I wrote that might be of use. The first is a ROS package called `sspp_simulator`, and the second is a python script called `change.py`
### sspp_simulator
sspp_simulator  takes the waypoints generated from the sspp_planner service and reformats them and publishes them to the RotorS simulator. After installing and building with `catkin build` you can run it with `roslaunch sspp_simulator main.launch` . It's worth looking over the launch file (`sspp_simulator/launch/main.launch`) to play with some of the parameters such as start_x, start_y, and so forth.

### change
`change.py` is a script that transforms .stl files to .pcd files for use with the algorithm. It requires open3d, which can be installed with
```
pip3 install open3d
```
To transform a file called airplane.stl to airplane.pcd you would do the following
```
python3 change.py airplane.stl
```
Be aware you may have to adjust the `scale_factor` variable in the code to ensure the model is appropriately sized for the path planning.