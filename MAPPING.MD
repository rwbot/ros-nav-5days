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

* Map.PGM
![enter image description here](https://github.com/rwbot/ros-nav-5days/blob/master/images/map.png?raw=true)
* Map.YAML
![
](https://github.com/rwbot/ros-nav-5days/blob/master/images/my_map_txt.png?raw=true)

The YAML File generated will contain the 6 following fields: 
* image: Name of the file containing the image of the generated Map. 
* resolution: Resolution of the map (in meters/pixel). origin: Coordinates of the lower-left pixel in the map. This coordinates are given in 2D (x,y). The third value indicates the rotation. If there's no rotation, the value will be 0. occupied_thresh: Pixels which have a value greater than this value will be considered as a completely occupied zone. free_thresh: Pixels which have a value smaller than this value will be considered as a completely free zone. negate: Inverts the colours of the Map. By default, white means completely free and black means completely occupied.




































#
<!--stackedit_data:
eyJoaXN0b3J5IjpbMjAzNTM3NTY3LC0xMzY3MTAyMDcsLTE3OT
Y4NTg4MjcsMTQ5Mzg1NDY3NV19
-->