## SLAM[](https://i-0b94a5b2534177362.robotigniteacademy.com/jupyter/notebooks/Mapping.ipynb#SLAM)

Simultaneous Localization and Mapping (SLAM). This is the name that defines the robotic problem of  **building a map of an unknown environment while simultaneously keeping track of the robot's location on the map that is being built**. This is basically the problem that Mapping is solving. The next Unit (Localization) is also involved, but we'll get there later.

So, summarizing,  **we need to do SLAM in order to create a Map for the robot**.

## The gmapping package[](https://i-0b94a5b2534177362.robotigniteacademy.com/jupyter/notebooks/Mapping.ipynb#The-gmapping-package)

The gmapping ROS package is an implementation of a specific SLAM algorithm called  _[gmapping](https://www.openslam.org/gmapping.html)_. This means that,  [somebody](http://wiki.ros.org/slam_gmapping)  has implemented the gmapping algorithm for you to use inside ROS, without having to code it yourself. So if you use the ROS Navigation stack, you only need to know (and have to worry about) how to configure gmapping for your specific robot (which is precisely what you'll learn in this Chapter).  
  
The gmapping package contains a ROS Node called  **slam_gmapping**, which allows you to create a 2D map using the laser and pose data that your mobile robot is providing while moving around an environment. This node  **basically reads data from the laser and the transforms of the robot, and turns it into an occupancy grid map**  (OGM).

So basically, what you've just done in the previous exercise was the following:

1.  You used a previously created configuration launch file (**_gmapping_demo.launch_**) to launch the  **gmapping**  package with the Kobuki robot.
2.  That launch file started a  **slam_gmapping node**  (from the gmapping package). Then you moved the robot around the room.
3.  Then ,the slam_gmapping node  **subscribed to the Laser (/kobuki/laser/scan) and the Transform Topics (/tf)**  in order to get the data it needs, and it built a map.
4.  The generated map is published during the whole process into the  **/map**  topic, which is the reason you could see the process of building the map with Rviz (because Rviz just visualizes topics).

The /map topic uses a message type of  **nav_msgs/OccupancyGrid**, since it is an OGM. Occupancy is represented as an integer in the range {0, 100}. With 0 meaning completely free, 100 meaning completely occupied, and the special value of -1 for completely unknown.

Amazing, right?  
  
Now, you may be worrying that you only had to do a roslaunch in order to have the robot generating the map.

-   What if your Kobuki does not have the laser at the center?
-   What if instead of a laser, you are using a Kinect?
-   What if you want to use the mapping with a different robot than Kobuki?

In order to be able to answer those questions, you still need to learn some things first.

Let's start by seeing what you can do with the Map you've just created.

## Saving the map[](https://i-0b94a5b2534177362.robotigniteacademy.com/jupyter/notebooks/Mapping.ipynb#Saving-the-map)

Another of the packages available in the ROS Navigation Stack is the  **map_server package**. This package provides the  **map_saver node**, which allows us to access the map data from a ROS Service, and save it into a file.

When you request the map_saver to save the current map, the map data is saved into two files: one is the YAML file, which contains the map metadata and the image name, and second is the image itself, which has the encoded data of the occupancy grid map.
```bash
rosrun map_server map_saver -f map_name
```
This command will get the map data from the map topic, and write it out into 2 files, **map_name.pgm** and **map_name.yaml**.

>**Note 1**: The -f attribute allows you to give the files a custom name. By default (if you don't use the -f attribute), the names of the file would be map.pgm and map.yaml.

>**Note 2**: Remember that, in order to be able to visualize the files generated through RViz, these files must be at the  **/home/user/catkin_ws/src**  directory. The files will be initially saved in the directory where you execute the command.

#### Map.PGM (Portable Gray Map) file is a file that represents a grayscale image

![enter image description here](https://github.com/rwbot/ros-nav-5days/blob/master/images/map.png?raw=true)

The image describes the occupancy state of each cell of the world in the color of the corresponding pixel.  **Whiter pixels are free**,  **blacker pixels are occupied**, and  **pixels in between are unknown**. Color and grayscale images are accepted, but most maps are gray (even though they may be stored as if in color). Thresholds in the YAML file are used to divide the three categories.

When communicated via ROS messages, occupancy is represented as an integer in the range [0,100], with  **0 meaning completely free and 100 meaning completely occupied, and the special value -1 for completely unknown**.

Image data is read in via SDL_Image; supported formats vary, depending on what SDL_Image provides on a specific platform. Generally speaking, most popular image formats are widely supported.

#### Map.YAML
![
](https://github.com/rwbot/ros-nav-5days/blob/master/images/my_map_txt.png?raw=true)

The YAML File generated will contain the 6 following fields: 
* **image**: Name of the file containing the image of the generated Map. 
* resolution: Resolution of the map (in meters/pixel). 
* **origin**: Coordinates of the lower-left pixel in the map. This coordinates are given in 2D (x,y). The third value indicates the rotation. If there's no rotation, the value will be 0. 
* **occupied_thresh**: Pixels which have a value greater than this value will be considered as a completely occupied zone. 
* **free_thresh**: Pixels which have a value smaller than this value will be considered as a completely free zone. **negate**: Inverts the colours of the Map. By default, white means completely free and black means completely occupied.


## Providing the map[¶](https://i-0b94a5b2534177362.robotigniteacademy.com/jupyter/notebooks/Mapping.ipynb#Providing-the-map)

Besides the map_saver node, the map_server package also provides the  **map_server node**. This node reads a map file from the disk and provides the map to any other node that requests it via a ROS Service.

Many nodes request to the  **map_server** the current map in which the robot is moving. This request is done, for instance, by the move_base node in order to get data from a map and use it to perform Path Planning, or by the localization node in order to figure out where in the map the robot is. You'll see examples of this usage in following chapters.

The service to call in order to get the map is:

-   **static_map - (nav_msgs/GetMap)**: Provides the map occupancy data through this service.
```bash
rosservice call /static_map "{}"
```

Apart from requesting the map through the service above, there are two latched topics that you can connect in order to get a ROS message with the map. The topics at which this node writes the map data are:

-   **map_metadata - (nav_msgs/MapMetaData):** Provides the map metadata through this topic.
-   **map -  (nav_msgs/OccupancyGrid):** Provides the map occupancy data through this topic.

>**NOTE**: When a topic is latched, it means that the last message published to that topic will be saved. That means, any node that listens to this topic in the future will get this last message, even if nobody is publishing to this topic anymore. In order to specify that a topic will be latched, you just have to set the  _latch_  attribute to true when creating the topic.

To launch the `map_server` node in order to provide information of a map given a map file, use the following command:
```
rosrun map_server map_server map_file.yaml
```
**Remember**: You must have created the map (the map_file.yaml file) previously with the gmapping node. You can't provide a map that you haven't created!

**provide_map_server.launch**
```xml
<launch>
    <arg name="map_file" default="/home/user/catkin_ws/src/test_map.yaml"/>
    <node pkg="map_server" name="map_server" type="map_server" args="$(arg map_file)"/>
</launch>
```
Echo of the **`/map_metadata`** topic:
![enter image description here](https://github.com/rwbot/ros-nav-5days/blob/master/images/echo_map_metadata.png?raw=true)
Echo of the **`/map`** topic:
![enter image description here](https://github.com/rwbot/ros-nav-5days/blob/master/images/echo_map.png?raw=true)

#### Service Client for **`/static_map`**
* **call_map_service.py**
```python
#! /usr/bin/env python

import rospy
from nav_msgs.srv import GetMap, GetMapRequest
import sys 

rospy.init_node('service_client') # Initialise a ROS node with the name service_client
rospy.wait_for_service('/static_map') # Wait for the service /static_map to be running
get_map_service = rospy.ServiceProxy('/static_map', GetMap) # Create the connection to the service
get_map = GetMapRequest() # Create an object of type GetMapRequest
result = get_map_service(get_map) # Call the service
print result # Print the result given by the service called
```
NOTE:
-   The map created is a  **static map**. This means that the map will always stay as it was when created . So when you create a Map, it will capture the environment as it is at the exact moment that the mapping process is being performed. If for any reason, the environment changes in the future, these changes won't appear on the map, hence it won't be valid anymore (or it won't correspond to the actual environment).  
      
-   The map that you created is a  **2D Map**. This means that, the obstacles that appear on the map don't have height. So if, for instance, you try to use this map to navigate with a drone, it won't be valid. There exist packages that allow you to generate 3D mappings, but this issue won't be covered in this Course. 


## Transforms
In the previous exercise you had the robot transforms properly configured, so you can trust that the laser readings will be correctly transformed into the Odom frame. But will this always be like this? NO, it doesn't have to.  
  
In order to be able to use the laser readings, we need to set a transform between the laser and the robot base, and add it to the transform tree. Transform? Transform tree? What are you talking about?  
  
Ok, let's clarify all this. So we have a Kobuki Robot with a laser mounted on it, right? But,  **in order to be able to use the laser data, we need to tell the robot WHERE (position and orientation) this laser is mounted in the robot**. This is what is called a  **_transform between frames_**.

A transform specifies how data expressed in a frame can be transformed into a different frame. For instance, if you detect an obstacle with the laser at 3 cm in the front, this means that it is 3 cm from the laser, but not from the center of the robot (that is usually called the  **_/base_link_**). To know the distance from the center of the robot, you need to  **_transform_  the 3 cm from the  _/laser_frame_  to the  _/base_link_**  frame (which is actually what the Path Planning system needs to know, what is the distance from the center of the robot to the obstacle).

Don't you think so? If we don't provide this information to the robot, when the laser detects an object, how can the robot know where this object is? Is it in front of the robot? Is it behind? Is it to the right? There's no way the robot can know it if we don't tell the robot the  **POSITION and the ORIENTATION**  of the laser regarding to the center of the robot. In order to do this, we need to do the following:  
  
First, we'll define two frames (coordinate frames), one at the center of the laser and another one at the center of the robot. For navigation, it is important that the center of the robot is placed at the  **rotational center**  of the robot. We'll name the laser frame as  _base_laser_  and the robot frame as  _base_link_.

Here you can see an scheme of how it would look like:
![](https://github.com/rwbot/ros-nav-5days/blob/master/images/tf_diagram.png?raw=true)

For instance, in the case of the Kobuki robot that you are using in this Chapter, the frames looks like this:
![](https://github.com/rwbot/ros-nav-5days/blob/master/images/base_laser_tfs.png?raw=true)

Now, we need to define a relationship (in terms of position and orientation) between the  _base_laser_  and the  _base_link_. For instance, we know that the  _base_laser frame_  is at a distance of 20 cm in the  _y_  axis and 10 cm in the  _x_axis referring the  _base_link frame_. Then we'll have to provide this relationship to the robot.  **This relationship between the position of the laser and the base of the robot is known in ROS as the TRANSFORM between the laser and the robot**.

For the slam_gmapping node to work properly, you will need to provide 2 transforms:

-   **the frame attached to laser -> base_link**: Usually a fixed value, broadcast periodically by a robot_state_publisher, or a tf static_transform_publisher.
-   **base_link -> odom**: Usually provided by the Odometry system

Since the robot needs to be able to access this information anytime, we will publish this information to a  **transform tree**. The transform tree is like a database where we can find information about all the transformations between the different frames (elements) of the robot.

You can visualize the transform tree of your running system anytime by using the following command:

```
rosrun tf view_frames
```
This command will generate a pdf file containing a graph with the transform tree of your system.
![enter image description here](https://github.com/rwbot/ros-nav-5days/blob/master/images/frames_pdf.png?raw=true)

Now, let's imagine you just mounted the laser on your robot, so the transform between your laser and the base of the robot is not set. What could you do? There are basically 2 ways of publishing a transform:

-   Use a  **static_transform_publisher**
-   Use a  **transform broadcaster**

In this Course, we'll use the **`static_transform_publisher`**, since it's the fastest way. The **`static_transform_publisher`** is a ready-to-use node that allows us to directly publish a transform by simply using the command line. The structure of the command is the next one:

```
static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
```
Where:

-   **x, y, z**  are the offsets in meters
-   **yaw, pitch, roll**  are the rotation in radians
-   **period_in_ms**  specifies how often to send the transform

You can also create a launch file that launches the command above, specifying the different values :

```xml
<launch>
 <node 
 pkg="tf" 
 type="static_transform_publisher" 
 name="name_of_node" 
 args="x y z yaw pitch roll frame_id child_frame_id period_in_ms">
 </node>
</launch>
```
---
  
Having shown how to create a transform broadcaster, let me tell you a secret. This is not something that you'll usually have to do. 
  
In the previous Chapter (Basic Concepts), you learned that the description of the robot model is made in the URDF files, right? Well, the  **publication of the transforms is also handled by the URDF files**. At least, this is the common use. There exist, though, some cases where you do have to publish a transform separately from the URDF files. For instance:

-   If you temporarily add a sensor to the robot. That is, you add a sensor that will be used during a few days, and then it will be removed again. In a case like this, instead of changing the URDF files of the robot (which will probably be more cumbersome), you'll just create a transform broadcaster to add the transform of this new sensor.  
    
-   You have a sensor that is external from your robot (and static). For instance, check the following scenario:
![enter image description here](https://github.com/rwbot/ros-nav-5days/blob/master/images/iri_wam.png?raw=true)


You have a robotic arm and, separated from it, you also have a Kinect camera, which provides information about the table to the robotic arm. In a case like this, you won't specify the transforms of the Kinect camera in the URDF files of the robot, since it's not a part of the robot. You'll also use a separated transform broadcaster.


---
---


# slam_gmapping node
The main task to create this launch file, as you may imagine, is to correctly set the parameters for the slam_gmapping node. This node is highly configurable and has lots of parameters you can change in order to improve the mapping performance. This parameters will be read from the ROS Parameter Server, and can be set either in the launch file itself or in a separated parameter files (YAML file). If you don't set some parameters, it will just take the default values. 
[Complete list of Gmapping Parameters](http://wiki.ros.org/gmapping)

### General Parameters
-   **base_frame (default: "base_link")**: Indicates the name of the frame attached to the mobile base.
-   **map_frame (default: "map")**: Indicates the name of the frame attached to the map.
-   **odom_frame (default: "odom")**: Indicates the name of the frame attached to the odometry system.
-   **map_update_interval (default: 5.0)**: Sets the time (in seconds) to wait until update the map.


### Laser Parameters
-   **maxRange (float)**: Sets the maximum range of the laser. Set this value to something slightly higher than the real sensor's maximum range.
-   **maxUrange (default: 80.0)**: Sets the maximum usable range of the laser. The laser beams will be cropped to this value.
-   **minimumScore (default: 0.0)**: Sets the minimum score to consider a laser reading good.


### Initial map dimensions and resolutions

-   **xmin (default: -100.0)**: Initial map size
-   **ymin (default: -100.0)**: Initial map size
-   **xmax (default: 100.0)**: Initial map size
-   **ymax (default: 100.0)**: Initial map size
-   **delta (default: 0.05)**: Sets the resolution of the map

### Other Parameters

-   **linearUpdate (default: 1.0)**: Sets the linear distance that the robot has to move in order to process a laser reading.
-   **angularUpdate (default: 0.5)**: Sets the angular distance that the robot has to move in order to process a laser reading.
-   **temporalUpdate (default: -1.0)**: Sets the time (in seconds) to wait between laser readings. If this value is set to -1.0, then this function is turned off.
-   **particles (default: 30)**: Number of particles in the filter
---
In the **_gmapping_demo.launch_** file, the parameters where loaded in the launch file itself, as you've seen. So you changed the parameters directly in the launch file. But this is not the only way you have to load parameters. In fact, parameters are usually loaded from an external file. This file that contains the parameters is usually a **YAML file**.  The syntax is: '****name_of_paramter: value_of_parameter****'

```yaml
base_frame: base_footprint
odom_frame: odom
map_update_interval: 3.0
maxUrange: 20.0
maxRange: 80.0
sigma: 0.05
kernelSize: 1
lstep: 0.05
astep: 0.05
iterations: 5
lsigma: 0.075
ogain: 3.0
lskip: 0
minimumScore: 200
srr: 0.01
srt: 0.02
str: 0.01
stt: 0.02
linearUpdate: 0.5
angularUpdate: 0.436
temporalUpdate: -1.0
resampleThreshold: 0.9
particles: 80

#xmin: -50.0
#ymin: -50.0
#xmax: 50.0
#ymax: 50.0
#make the starting size small for the benefit of the Android client's memory...

xmin: -1.0
ymin: -1.0
xmax: 1.0
ymax: 1.0
delta: 0.05
llsamplerange: 0.01
llsamplestep: 0.01
lasamplerange: 0.005
lasamplestep: 0.005
```
  

So, you can also write all the parameters in a YAML file, and then load this file (and the parameters) in the launch file just by adding the following line inside the **<node>** tag:
```xml
<rosparam file="$(find my_mapping_launcher)/params/gmapping_params.yaml" command="load" />
```
This will have the exact same result as if the parameters are loaded directly through the launch file. And as you will see, it is a much more clean way to do it.



# Build a Map Using Logged Data

Until now we've seen how to build a map by moving the robot in real-time. But this is not the only way of creating a map, of course! As you already know, the process of building a map is based on reading the data that is being published in the laser and the transform topics. That's why we were moving the robot around, to publish the data that the robot was getting in real-time while moving. But if we just need some data being published on those topics... doesn't it come to your mind another obvious way of creating a map? That's right! You could just use a bag file in order to publish data on those topics, and therefore build a map.  

 [ROS Wiki: rosbag](http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData)
 
In order to build a map using logged data, you will have to follow these 2 steps (which are divided into sub-steps):

### 1. Create the bag file

In order to create a proper bag file for Mapping, you'll need to follow the next steps:

a) First of all, launch your keyboard teleop to start moving the robot:
```
roslaunch pkg_name keyboard_teleop_launch_file.launch
```
b) Make sure that the robot is publishing its laser data and the tfs.

```
rostopic list
```
c) Start recording scans and transforms (note that the scan topic may vary from robot to robot):

[esto no puede funcionar porque no estas grabando la odometria]

```
rosbag record -O mylaserdata /laser_topic /tf_topic
```

This will start writing a file in the current directory called  `mylaserdata.bag`.

d) Drive the robot around. General advice:

-   Try to limit fast rotations, as they are hardest on the scan-matcher. It helps to lower the speed.
-   Visualize what the robot "sees" with its laser; if the laser can't see it, it won't be in the map.
-   Loop closure is the hardest part; when closing a loop, be sure to drive another 5-10 meters to get plenty of overlap between the start and end of the loop.

f) Kill the rosbag instance, and note the name of the file that was created.

g) You can check if a bag file has been created properly by: 
```
rosbag info name_of_bag_file.bag
```


### 2. Build the Map[](https://i-0dfe0fc43dcf0c827.robotigniteacademy.com/jupyter/notebooks/Mapping.ipynb#2.-Build-the-Map)

Once we have the bag file, we're ready to build the map! In order to do see, you'll need to follow the next steps:

a) Start the slam_gmapping node, which will take in laser scans (in this case, on the **`/kobuki/laser/scan`** topic) and produce a map:

```
rosrun gmapping slam_gmapping scan:=kobuki/laser/scan
```
b) Play the bag file to provide data to the slam_gmapping node:

```
rosbag play name_of_bag_file_created_in_step_1
```
Wait for rosbag to finish and exit.

c) Save the created map using the map_saver node as shown in previous sections:

```
rosrun map_server map_saver -f map_name
```
Now you will have 2 files (map_name.pgm and map_name.yaml) that should look the same as the ones you created in Exercise 1.3.










#
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTE5MDkyMzg1NTIsLTExMzAxNjIwMjMsLT
EzMzkwMzY4NTAsLTE0MDU5MDMxNTMsLTE5Nzg3MjMyNjYsLTg5
NzQ2MDI0MCwtMjEwNTY3NjI4MSwtOTAxNTE4NTUyLC01ODU5OD
k2OTYsLTE3OTUzNTU4NDgsODcxMzU0NTkxLC0xMzY3MTAyMDcs
LTE3OTY4NTg4MjcsMTQ5Mzg1NDY3NV19
-->