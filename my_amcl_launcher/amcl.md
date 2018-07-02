## Monte Carlo Localization (MCL)[](https://i-0e38676f9310f7ae3.robotigniteacademy.com/jupyter/notebooks/Localization.ipynb#Monte-Carlo-Localization-%28MCL%29)

Because the robot may not always move as expected, it generates many random guesses as to where it is going to move next. These guesses are known as particles. Each particle contains a full description of a possible future pose. When the robot observes the environment it's in (via sensor readings), it discards particles that don't match with these readings, and generates more particles close to those that look more probable. This way, in the end, most of the particles will converge in the most probable pose that the robot is in. So **the more you move, the more data you'll get from your sensors, hence the localization will be more precise**. These particles are those **_arrows_** that you saw in RViz in the previous exercise. Amazing, right?

This is known as the Monte Carlo Localization (MCL) algorithm, or also particle filter localization.

## The AMCL package[](https://i-0e38676f9310f7ae3.robotigniteacademy.com/jupyter/notebooks/Localization.ipynb#The-AMCL-package)

The AMCL (Adaptive Monte Carlo Localization) package provides the **amcl node**, which uses the MCL system in order to track the localization of a robot moving in a 2D space. This node **subscribes to the data of the laser, the laser-based map, and the transformations of the robot, and publishes its estimated position in the map**. On startup, the amcl node initializes its particle filter according to the parameters provided.

**NOTE**: As you may have noticed, in order to name this ROS package (and node), the word **Adaptive** has been added to the Monte Carlo Localization algorithm. This is because, in this node, we will be able to configure (adapt) some of the parameters that are used in this algorithm. You'll learn more about these parameters later on in this chapter.

So, basically, what you've done in the previous exercise was the following:  

-   First, you launched an **amcl node** using the preconfigured **_amcl_demo.launch_** file.
-   Second, you set up an initial pose by using the 2D Pose Estimate tool (which published that pose to the **/initialpose** topic).
-   Then, you started moving the robot around the room, and the **amcl node** began reading the data published into the **laser topic (/scan)**, the **map topic(/map)**, and the **transform topic (/tf)**, and published the estimated pose where the robot was in to the **/amcl_pose** and the **/particlecloud** topics.
-   Finally, via RViz, you accesed the data being published by this node into the **/particlecloud** topic, so you were able to vizualize it, thanks to the cloud of "arrows," which were indicating the most probable position the robot was in, and its orientation.
---
Now you may be thinking... but where does this node get the map from? And that's a great question!

In the previous unit, you learned about the **map_server** node, which allows you to provide the data of a map from the map's file. Do you remember? If not, I suggest you go back and take a quick look in order to refresh your memory.

Back again? So basically, what we're doing here is to call that functionality in order to provide the map data to the amcl node. And we're doing it through the **_amcl_demo.launch_** file that you launched back in Exercise 3.1. If you take a look at this file, you'll see at the top of it, there is a section like this:

```xml
  <!-- Run the map server -->
  <arg name="map_file" default="$(find husky_navigation)/maps/my_map.yaml"/>
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />
 ```
Here, the file is launching a **map_server** node, which will take the provided map file (arg **map_file**) and turn it into map data. Then, it will publish this data to the **/map** topic, which will be used by the **amcl** node to perform localization.


## Hardware Requirements

As we saw in the previous chapter (Mapping), **configuration is also very important to properly localizing the robot in the environment**.

In order to get a proper Robot localization, we need to fullfil 3 basic requirements:

-   Provide Good **Laser Data**
-   Provide Good **Odometry Data**
-   Provide Good Laser-Based **Map Data**

## Transforms

As we also saw in the previous chapter (Mapping), we need to be publishing a correct **transform** between the laser frame and the base of the robot's frame. This is pretty obvious, since as you've already learned, the robot **uses the laser readings in order to constantly re-calculate it's localization**.

More specifically, the amcl node has these 2 requirements regarding the transformations of the robot:

-   amcl transforms incoming laser scans to the odometry frame (~odom_frame_id). So, there must be a path through the tf tree from the frame in which the laser scans are published to the odometry frame.
-   amcl looks up the transform between the laser's frame and the base frame (~base_frame_id), and latches it forever. So **amcl cannot handle a laser that moves with respect to the base.**

```
rosrun tf view_frames
```

---

## Creating a launch file for the AMCL node

As you saw in the Mapping Chapter, you also need to have a launch file in order to start the amcl node. This node is also highly customizable and we can configure many parameters in order to improve its performance. These parameters can be set either in the launch file itself or in a separate parameters file (YAML file). You can have a look at a complete list of all of the parameters that this node has here: [http://wiki.ros.org/amcl](http://wiki.ros.org/amcl)  
  
Let's have a look at some of the most important ones:

##### General Parameters

-   **odom_model_type (default: "diff")**: It puts the odometry model to use. It can be "diff," "omni," "diff-corrected," or "omni-corrected."
-   **odom_frame_id (default: "odom")**: Indicates the frame associated with odometry.
-   **base_frame_id (default: "base_link")**: Indicates the frame associated with the robot base.
-   **global_frame_id (default: "map")**: Indicates the name of the coordinate frame published by the localization system.
-   **use_map_topic (default: false)**: Indicates if the node gets the map data from the topic or from a service call.

As discussed previously, the amcl node needs to get data from the **/map** topic in order to work properly, so you'll need to publish map information into the proper topic.

#### Filter Parameters
These parameters will allow you to configure the way that the particle filter performs.

-   **min_particles (default: 100)**: Sets the minimum allowed number of particles for the filter.
-   **max_particles (default: 5000)**: Sets the maximum allowed number of particles for the filter.
-   **kld_err (default: 0.01)**: Sets the maximum error allowed between the true distribution and the estimated distribution.
-   **update_min_d (default: 0.2)**: Sets the linear distance (in meters) that the robot has to move in order to perform a filter update.
-   **update_min_a (default: π/6.0)**: Sets the angular distance (in radians) that the robot has to move in order to perform a filter update.
-   **resample_interval (default: 2)**: Sets the number of filter updates required before resampling.
-   **transform_tolerance (default: 0.1)**: Time (in seconds) with which to post-date the transform that is published, to indicate that this transform is valid into the future.
-   **gui_publish_rate (default: -1.0)**: Maximum rate (in Hz) at which scans and paths are published for visualization. If this value is -1.0, this function is disabled.

#### Laser Parameters

These parameters will allow you to configure the way the amcl node interacts with the laser.

-   **laser_min_range (default: -1.0)**: Minimum scan range to be considered; -1.0 will cause the laser's reported minimum range to be used.
-   **laser_max_range (default: -1.0)**: Maximum scan range to be considered; -1.0 will cause the laser's reported maximum range to be used.
-   **laser_max_beams (default: 30)**: How many evenly-spaced beams in each scan to be used when updating the filter.
-   **laser_z_hit (default: 0.95)**: Mixture weight for the z_hit part of the model.
-   **laser_z_short (default: 0.1)**: Mixture weight for the z_short part of the model.
-   **laser_z_max (default: 0.05)**: Mixture weight for the z_max part of the model.
-   **laser_z_rand (default: 0.05)**: Mixture weight for the z_rand part of the model.














/
