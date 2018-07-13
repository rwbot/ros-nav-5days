
# Solutions for Unit 2 Mapping



#The /map topic uses a message type of nav_msgs/OccupancyGrid, since it is an OGM. Occupancy is represented as an integer in the range {0, 100}. With 0 meaning completely free, 100 meaning completely occupied, and the special value of -1 for completely unknown.


Saving Map

#Save map The -f attribute allows you to give the files a custom name. By default are map.pgm and map.yaml.
rosrun map_server map_saver -f name_of_map




The YAML File generated will contain the 6 following fields:
image: Name of the file containing the image of the generated Map.
resolution: Resolution of the map (in meters/pixel).
origin: Coordinates of the lower-left pixel in the map. This coordinates are given in 2D (x,y). The third value indicates the rotation. If there's no rotation, the value will be 0. occupied_thresh: Pixels which have a value greater than this value will be considered as a completely occupied zone.
free_thresh: Pixels which have a value smaller than this value will be considered as a completely free zone.
negate: Inverts the colours of the Map. By default, white means completely free and black means completely occupied.


Providing Map

static_map (nav_msgs/GetMap): Provides the map occupancy data through this ser
map_metadata (nav_msgs/MapMetaData): Provides the map metadata through this topic. map (nav_msgs/OccupancyGrid): Provides the map occupancy data through this topic.
NOTE: When a topic is latched, it means that the last message published to that topic will be saved. That means, any node that listens to this topic in the future will get this last message, even if nobody is publishing to this topic anymore. In order to specify that a topic will be latched, you just have to set the latch attribute to true when creating the topic.



Launching Map Server Node
Remember: You must have created the map (the map_file.yaml file) previously with the gmapping node. You can't provide a map that you haven't created!

$$$rosrun map_server map_server map_file.yaml

static_map (nav_msgs/GetMap): Provides the map occupancy data through this service.

$$$rosservice call /static_map "{}"
























#### Exercise 2.5
For this exercise, we will assume that our map file is called my_map.yaml, and that it is placed into the catkin_ws/src directory.
Launch File: move_robot.launch
```
#launch the map_server node in order to provide the map you've created.
<launch>
    <arg name="map_file" default="/home/user/catkin_ws/src/my_map.yaml"/>
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />
</launch>
```


---
Solution Exercise 2.7
Create a Service Client that calls to the service /static_map.
Launch File: call_map_service.launch
```
<launch>
    <node pkg="get_map_data" type="call_map_service.py" name="service_client" output="screen" />
</launch>
```


Python File: call_map_service.py
This Service Client will call the /static_map service in order to get the map data, and then it will print the dimensions and resolution of the map in the screen

```
#! /usr/bin/env python
import rospy
from nav_msgs.srv import GetMap, GetMapRequest
import sys
# Initialise a ROS node with the name service_client​
rospy.init_node('service_client')
# Wait for the service /static_map to be running
rospy.wait_for_service('/static_map')
# Create the connection to the service
get_map_service = rospy.ServiceProxy('/static_map', GetMap)
# Create an object of type GetMapRequest
get_map = GetMapRequest()
# Call the service
result = get_map_service(get_map)
# Print the result given by the service called
print result
```


* The map that you created is a static map. This means that the map will always stay as it was when you created it. So when you create a Map, it will capture the environment as it is at the exact moment that the mapping process is being performed. If for any reason, the environment changes in the future, these changes won't appear on the map, hence it won't be valid anymore (or it won't correspond to the actual environment).
* The map that you created is a 2D Map. This means that, the obstacles that appear on the map don't have height. So if, for instance, you try to use this map to navigate with a drone, it won't be valid. There exist packages that allow you to generate 3D mappings, but this issue won't be covered in this Course. If you're interested in this topic, you can have a look at the following link: http://wiki.ros.org/rtabmap_ros/Tutorials/MappingAndNavigationOnTurtlebot

-
Solution Exercise 2.10
Create a package and a launch file in order to launch a static_transform_publisher node. This node should publish the transform between the Kinect camera mounted on the robot and the base link of the robot.
Launch File: pub_static_tf.launch
```
<launch>
    <node pkg="tf" type="static_transform_publisher" name="static_tf_node"
          args="1 0 0 0 0 0 base_link kinect_link 30">
    </node>
</launch>
```

-
Solution Exercise 2.11
Launch File: my_gmapping_launch.launch
```
<launch>
  <arg name="scan_topic"  default="kobuki/laser/scan" />
  <arg name="base_frame"  default="base_footprint"/>
  <arg name="odom_frame"  default="odom"/>
​
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <param name="base_frame" value="$(arg base_frame)"/>
    <param name="odom_frame" value="$(arg odom_frame)"/>
    <param name="map_update_interval" value="15.0"/>
    <param name="maxUrange" value="6.0"/>
    <param name="maxRange" value="8.0"/>
    <param name="sigma" value="0.05"/>
    <param name="kernelSize" value="1"/>
    <param name="lstep" value="0.05"/>
    <param name="astep" value="0.05"/>
    <param name="iterations" value="5"/>
    <param name="lsigma" value="0.075"/>
    <param name="ogain" value="3.0"/>
    <param name="lskip" value="0"/>
    <param name="minimumScore" value="200"/>
    <param name="srr" value="0.01"/>
    <param name="srt" value="0.02"/>
    <param name="str" value="0.01"/>
    <param name="stt" value="0.02"/>
    <param name="linearUpdate" value="0.5"/>
    <param name="angularUpdate" value="0.436"/>
    <param name="temporalUpdate" value="-1.0"/>
    <param name="resampleThreshold" value="0.5"/>
    <param name="particles" value="80"/>
  <!--
    <param name="xmin" value="-50.0"/>
    <param name="ymin" value="-50.0"/>
    <param name="xmax" value="50.0"/>
    <param name="ymax" value="50.0"/>
  make the starting size small for the benefit of the Android client's memory...
  -->
    <param name="xmin" value="-1.0"/>
    <param name="ymin" value="-1.0"/>
    <param name="xmax" value="1.0"/>
    <param name="ymax" value="1.0"/>
​
    <param name="delta" value="0.05"/>
    <param name="llsamplerange" value="0.01"/>
    <param name="llsamplestep" value="0.01"/>
    <param name="lasamplerange" value="0.005"/>
    <param name="lasamplestep" value="0.005"/>
    <remap from="scan" to="$(arg scan_topic)"/>
  </node>
</launch>
```

-
Solution Exercise 2.12
Exercise 2.12
Launch File: my_gmapping_launch.launch
```
<launch>
  <arg name="scan_topic"  default="kobuki/laser/scan" />
  <arg name="base_frame"  default="base_footprint"/>
  <arg name="odom_frame"  default="odom"/>
​
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <param name="base_frame" value="$(arg base_frame)"/>
    <param name="odom_frame" value="$(arg odom_frame)"/>
    <param name="map_update_interval" value="5.0"/>
    <param name="maxUrange" value="2.0"/>
    <param name="maxRange" value="8.0"/>
    <param name="sigma" value="0.05"/>
    <param name="kernelSize" value="1"/>
    <param name="lstep" value="0.05"/>
    <param name="astep" value="0.05"/>
    <param name="iterations" value="5"/>
    <param name="lsigma" value="0.075"/>
    <param name="ogain" value="3.0"/>
    <param name="lskip" value="0"/>
    <param name="minimumScore" value="200"/>
    <param name="srr" value="0.01"/>
    <param name="srt" value="0.02"/>
    <param name="str" value="0.01"/>
    <param name="stt" value="0.02"/>
    <param name="linearUpdate" value="0.5"/>
    <param name="angularUpdate" value="0.436"/>
    <param name="temporalUpdate" value="-1.0"/>
    <param name="resampleThreshold" value="0.5"/>
    <param name="particles" value="80"/>
  <!--
    <param name="xmin" value="-50.0"/>
    <param name="ymin" value="-50.0"/>
    <param name="xmax" value="50.0"/>
    <param name="ymax" value="50.0"/>
  make the starting size small for the benefit of the Android client's memory...
  -->
    <param name="xmin" value="-1.0"/>
    <param name="ymin" value="-1.0"/>
    <param name="xmax" value="1.0"/>
    <param name="ymax" value="1.0"/>
​
    <param name="delta" value="0.05"/>
    <param name="llsamplerange" value="0.01"/>
    <param name="llsamplestep" value="0.01"/>
    <param name="lasamplerange" value="0.005"/>
    <param name="lasamplestep" value="0.005"/>
    <remap from="scan" to="$(arg scan_topic)"/>
  </node>
</launch>
```


-
Solution Exercise 2.13
Launch File: my_gmapping_launch.launch
```
<launch>
  <arg name="scan_topic"  default="kobuki/laser/scan" />
  <arg name="base_frame"  default="base_footprint"/>
  <arg name="odom_frame"  default="odom"/>
​
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <param name="base_frame" value="$(arg base_frame)"/>
    <param name="odom_frame" value="$(arg odom_frame)"/>
    <param name="map_update_interval" value="5.0"/>
    <param name="maxUrange" value="6.0"/>
    <param name="maxRange" value="8.0"/>
    <param name="sigma" value="0.05"/>
    <param name="kernelSize" value="1"/>
    <param name="lstep" value="0.05"/>
    <param name="astep" value="0.05"/>
    <param name="iterations" value="5"/>
    <param name="lsigma" value="0.075"/>
    <param name="ogain" value="3.0"/>
    <param name="lskip" value="0"/>
    <param name="minimumScore" value="200"/>
    <param name="srr" value="0.01"/>
    <param name="srt" value="0.02"/>
    <param name="str" value="0.01"/>
    <param name="stt" value="0.02"/>
    <param name="linearUpdate" value="0.5"/>
    <param name="angularUpdate" value="0.436"/>
    <param name="temporalUpdate" value="-1.0"/>
    <param name="resampleThreshold" value="0.5"/>
    <param name="particles" value="80"/>
  <!--
    <param name="xmin" value="-50.0"/>
    <param name="ymin" value="-50.0"/>
    <param name="xmax" value="50.0"/>
    <param name="ymax" value="50.0"/>
  make the starting size small for the benefit of the Android client's memory...
  -->
    <param name="xmin" value="-100.0"/>
    <param name="ymin" value="-100.0"/>
    <param name="xmax" value="100.0"/>
    <param name="ymax" value="100.0"/>
​
    <param name="delta" value="0.05"/>
    <param name="llsamplerange" value="0.01"/>
    <param name="llsamplestep" value="0.01"/>
    <param name="lasamplerange" value="0.005"/>
    <param name="lasamplestep" value="0.005"/>
    <remap from="scan" to="$(arg scan_topic)"/>
  </node>
</launch>
```

-
Solution Exercise 2.15
Launch File: my_gmapping_launch.launch
```
<launch>
     <arg name="scan_topic" default="/kobuki/laser/scan" />

   <!-- Defining parameters for slam_gmapping node -->
​
     <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping"
     output="screen">

       <rosparam file="$(find my_mapping_launcher)/params/gmapping_params.yaml" command="load" />

       <remap from="scan" to="$(arg scan_topic)"/>

     </node>

</launch>
```

Params File: gmapping_params.yaml
```
base_frame: base_footprint
odom_frame: odom
map_update_interval: 5.0
maxUrange: 6.0
maxRange: 8.0
​
minimumScore: 200
​
linearUpdate: 0.5
angularUpdate: 0.436
temporalUpdate: -1.0
resampleThreshold: 0.5
particles: 80
xmin: -1.0
ymin: -1.0
xmax: 1.0
ymax: 1.0
​
delta: 0.05
llsamplerange: 0.01
llsamplestep: 0.01
lasamplerange: 0.005
lasamplestep: 0.005
```



























> The overriding design goal for Markdown's
> formatting syntax is to make it as readable


| Plugin | README |
| ------ | ------ |
| Dropbox | [plugins/dropbox/README.md][PlDb] |
| Github | [plugins/github/README.md][PlGh] |


```sh
docker run -d -p 8000:8080 --restart="always" <youruser>/dillinger:${package.json.version}
```

# 1 Hash
## 2 Hash
### 3 Hash
#### 4 Hash

See [LINK](URL)



Dash Under = Heading
-

* Asterick = bulletpoint


***In 3 Astericks***
**In 2 Astericks**
*In 1 Asterick*
