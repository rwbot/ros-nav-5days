# The Local Planner

Once the global planner has calculated the path to follow, this path is sent to the local planner. The local planner, then, will execute each segment of the global plan (let's imagine the local plan as a smaller part of the global plan). So,  **given a plan to follow (provided by the global planner) and a map, the local planner will provide velocity commands in order to move the robot**.

Unlike the global planner, the  **local planner monitors the odometry and the laser data**, and chooses a collision-free local plan (let's imagine the local plan as a smaller part of the global plan) for the robot. So, the local planner  **can recompute the robot's path on the fly**  in order to keep the robot from striking objects, yet still allowing it to reach its destination.

Once the local plan is calculated, it is published into a topic named  **/local_plan**. The local planner also publishes the portion of the global plan that it is attemting to follow into the topic  **/global_plan**.


As for the global planner, different types of local planners also exist. Depending on your setup (the robot you use, the environment it navigates, etc.) and the type of performance you want, you will use one or another. Let's have a look at the most important ones.

### base_local_planner:  [http://wiki.ros.org/base_local_planner](http://wiki.ros.org/base_local_planner)

The base local planner provides implementations of the  _Trajectory Rollout_  and the  _Dynamic Window Approach (DWA)_  algorithms in order to calculate and execute a global plan for the robot.

Summarizing, the basic idea of how this algorithms works is as follows:

-   Discretely sample from the robot's control space
-   For each sampled velocity, perform forward simulations from the robot's current state to predict what would happen if the sampled velocity was applied.
-   Evaluate each trajectory resulting from the forward simulation.
-   Discard illegal trajectories.
-   Pick the highest-scoring trajectory and send the associated velocities to the mobile base.
-   Rinse and Repeat.

DWA differs from Trajectory Rollout in how the robot's space is sampled. Trajectory Rollout samples are from the set of achievable velocities over the entire forward simulation period given the acceleration limits of the robot, while DWA samples are from the set of achievable velocities for just one simulation step given the acceleration limits of the robot.

DWA is a more efficient algorithm because it samples a smaller space, but may be outperformed by Trajectory Rollout for robots with low acceleration limits because DWA does not forward simulate constant accelerations. In practice, DWA and Trajectory Rollout perform similarly, so  **it's recommended to use DWA because of its efficiency gains**.

The DWA algorithm of the base local planner has been improved in a new local planner separated from this one. That's the DWA local planner we'll see next.

### dwa_local_planner

The DWA local planner provides an implementation of the  _Dynamic Window Approach_  algorithm. It is basically a re-write of the base local planner's DWA (Dynamic Window Approach) option, but the code is a lot cleaner and easier to understand, particularly in the way that the trajectories are simulated.  
  
So, for applications that use the DWA approach for local planning, the dwa_local_planner is probaly the best choice. This is the  **most commonly used option**.

### eband_local_planner:  [http://wiki.ros.org/eband_local_planner](http://wiki.ros.org/eband_local_planner)

The eband local planner implements the  _Elastic Band_  method in order to calculate the local plan to follow.

### teb_local_planner:  [http://wiki.ros.org/teb_local_planner](http://wiki.ros.org/teb_local_planner)

The teb local planner implements the  _Timed Elastic Band_  method in order to calculate the local plan to follow.

## Setting the local planner

As for the global planner, you can also select which local planner you want to use. This is also done in the **move_base node parameters file**, by adding one of the following lines:
```yaml
base_local_planner: "base_local_planner/TrajectoryPlannerROS" # Sets the Trajectory Rollout algorithm from base local                                                                 planner
base_local_planner: "dwa_local_planner/DWAPlannerROS" # Sets the dwa local planner
base_local_planner: "eband_local_planner/EBandPlannerROS" # Sets the eband local planner
base_local_planner: "teb_local_planner/TebLocalPlannerROS" # Sets the teb local planner
```

# Local Planner Parameters
the local planner also has its own parameters. These parameters will be different depending on the local planner you use. In this course, we'll be focusing on the DWA local planner parameters, since it's the most common choice. 


## DWA Local Planner Parameters

The most important parameters for the DWA local planner are the following:

#### Robot Configuration Parameters

-   **/acc_lim_x (default: 2.5)**: The x acceleration limit of the robot in meters/sec^2
-   **/acc_lim_th (default: 3.2)**: The rotational acceleration limit of the robot in radians/sec^2
-   **/max_trans_vel (default: 0.55)**: The absolute value of the maximum translational velocity for the robot in m/s
-   **/min_trans_vel (default: 0.1)**: The absolute value of the minimum translational velocity for the robot in m/s
-   **/max_vel_x (default: 0.55)**: The maximum x velocity for the robot in m/s.
-   **/min_vel_x (default: 0.0)**: The minimum x velocity for the robot in m/s, negative for backwards motion.
-   **/max_rot_vel (default: 1.0)**: The absolute value of the maximum rotational velocity for the robot in rad/s
-   **/min_rot_vel (default: 0.4)**: The absolute value of the minimum rotational velocity for the robot in rad/s

#### Goal Tolerance Parameters
-   **/yaw_goal_tolerance (double, default: 0.05)**: The tolerance, in radians, for the controller in yaw/rotation when achieving its goal
-   **/xy_goal_tolerance (double, default: 0.10)**: The tolerance, in meters, for the controller in the x and y distance when achieving a goal
-   **/latch_xy_goal_tolerance (bool, default: false)**: If goal tolerance is latched, if the robot ever reaches the goal xy location, it will simply rotate in place, even if it ends up outside the goal tolerance while it is doing so.

#### Forward Simulation Parameters

-   **/sim_time (default: 1.7)**: The amount of time to forward-simulate trajectories in seconds
-   **/sim_granularity (default: 0.025)**: The step size, in meters, to take between points on a given trajectory
-   **/vx_samples (default: 3)**: The number of samples to use when exploring the x velocity space
-   **/vy_samples (default: 10...but for diff-drive:0)**: The number of samples to use when exploring the y velocity space. Differential drive robots do not move in the Y space
-   **/vtheta_samples (default: 20)**: The number of samples to use when exploring the theta velocity space

#### Trajectory Scoring Parameters
-   **/path_distance_bias (default: 32.0)**: The weighting for how much the controller should stay close to the path it was given
-   **/goal_distance_bias (default: 24.0)**: The weighting for how much the controller should attempt to reach its local goal; also controls speed
-   **/occdist_scale (default: 0.01)**: The weighting for how much the controller should attempt to avoid obstacles

* **dwa_local_planners_params.yaml**
```yaml
DWAPlannerROS:

# Robot Configuration Parameters - Kobuki
  max_vel_x: 0.5  # 0.55
  min_vel_x: 0.0

  max_vel_y: 0.0  # diff drive robot
  min_vel_y: 0.0  # diff drive robot

  max_trans_vel: 0.5 # choose slightly less than the base's capability
  min_trans_vel: 0.1  # this is the min trans velocity when there is negligible rotational velocity
  trans_stopped_vel: 0.1

  # Warning!
  #   do not set min_trans_vel to 0.0 otherwise dwa will always think translational velocities
  #   are non-negligible and small in place rotational velocities will be created.

  max_rot_vel: 5.0  # choose slightly less than the base's capability
  min_rot_vel: 0.4  # this is the min angular velocity when there is negligible translational velocity
  rot_stopped_vel: 0.4

  acc_lim_x: 1.0 # maximum is theoretically 2.0, but we
  acc_lim_theta: 2.0
  acc_lim_y: 0.0      # diff drive robot

# Goal Tolerance Parameters
  yaw_goal_tolerance: 0.3  # 0.05
  xy_goal_tolerance: 0.15  # 0.10
  # latch_xy_goal_tolerance: false

# Forward Simulation Parameters
  sim_time: 1.0       # 1.7
  vx_samples: 6       # 3
  vy_samples: 1       # diff drive robot, there is only one sample
  vtheta_samples: 20  # 20

# Trajectory Scoring Parameters
  path_distance_bias: 64.0      # 32.0   - weighting for how much it should stick to the global path plan
  goal_distance_bias: 24.0      # 24.0   - wighting for how much it should attempt to reach its goal
  occdist_scale: 0.5            # 0.01   - weighting for how much the controller should avoid obstacles
  forward_point_distance: 0.325 # 0.325  - how far along to place an additional scoring point
    stop_time_buffer: 0.2         # 0.2    - amount of time a robot must stop in before colliding for a valid traj.
  scaling_speed: 0.25           # 0.25   - absolute velocity at which to start scaling the robot's footprint
  max_scaling_factor: 0.2       # 0.2    - how much to scale the robot's footprint when at speed.

# Oscillation Prevention Parameters
  oscillation_reset_dist: 0.05  # 0.05   - how far to travel before resetting oscillation flags

# Debugging
  publish_traj_pc : true
  publish_cost_grid_pc: true
  global_frame_id: odom


# Differential-drive robot configuration - necessary?
#  holonomic_robot: false
```

---

# Local Costmap

The first thing you need to know is that the  **local planner uses the local costmap in order to calculate local plans**.

Unlike the global costmap, the local costmap is created directly from the robot's sensor readings. Given a width and a height for the costmap (which are defined by the user), it keeps the robot in the center of the costmap as it moves throughout the environment, dropping obstacle information from the map as the robot moves.

**The local costmap does detect new objects that appear in the simulation, while the global costmap doesn't**.

This happens because the global costmap is created from a static map file. This means that the costmap won't change, even if the environment does. The local costmap, instead, is created from the robot's sensor readings, so it will always keep updating with new readings from the sensors.

Since the global costmap and the local costmap don't have the same behavior, the parameters file must also be different. Let's have a look at the most important parameters that we need to set for the local costmap.

# Local Costmap Parameters

The parameters you need to know are the following:

-   **global_frame (default: /map)**: The global frame for the costmap to operate in. In the local costmap, this parameter has to be set to "/odom".
-   **static_map (default: false)** : Whether or not to use a static map to initialize the costmap. In the local costmap, this parameter has to be set to "false."
-   **rolling_window (default: true)**: Whether or not to use a rolling window version of the costmap. If the static_map parameter is set to true, this parameter must be set to false. In the local costmap, this parameter has to be set to "true."
-   **width (default: 10)**: The width of the costmap.
-   **heigth (default: 10)**: The height of the costmap.
-   **update_frequency (default: 5.0)**: The frequency in Hz for the map to be updated.
-   **plugins**: Sequence of plugin specifications, one per layer. Each specification is a dictionary with a name and type fields. The name is used to define the parameter namespace for the plugin.

So, by setting the  **static_map**  paramter to false, and the  **rolling_window**  parameter to true, we are indicating that we don't want the costmap to be initialized from a static map (as we did with the global costmap), but to be built from the robot's sensor readings.  
Also, since we won't have any static map, the  **global_frame**  parameter needs to be set to  **odom**.  
Finally, we also need to set a  **width**  and a  **height**  for the costmap, because in this case, it can't get these values from a static map.

### Plugins/Layers
Just as we saw for the global costmap, layers can also be added to the local costmap. In the case of the local costmap, you will usually add these 2 layers:

-   **costmap_2d::ObstacleLayer**: Used for obstacle avoidance.
-   **costmap_2d::InflationLayer**: Used to inflate obstacles.

```yaml
plugins:
 - {name: obstacle_layer,      type: "costmap_2d::ObstacleLayer"}
 - {name: inflation_layer,     type: "costmap_2d::InflationLayer"}
```
**VERY IMPORTANT:** Note that the  **obstacle layer**  uses different plugins for the  **local costmap**  and the  **global costmap**. For the local costmap, it uses the  **costmap_2d::ObstacleLayer**, and for the global costmap it uses the  **costmap_2d::VoxelLayer**. This is very important because it is a common error in Navigation to use the wrong plugin for the obstacle layers.

As you've already seen through the exercises, the local costmap keeps updating itself . These update cycles are made at a rate specified by the  **update_frequency**  parameter. Each cycle works as follows:

-   Sensor data comes in.
-   Marking and clearing operations are performed.
-   The appropriate cost values are assigned to each cell.
-   Obstacle inflation is performed on each cell with an obstacle. This consists of propagating cost values outwards from each occupied cell out to a specified inflation radius.

---
## Marking and Clearing

The costmap automatically subscribes to the sensor topics and updates itself according to the data it receives from them. Each sensor is used to either  **mark**  (insert obstacle information into the costmap),  **clear**  (remove obstacle information from the costmap), or both.

A  **marking**  operation is just an index into an array to change the cost of a cell.  
A  **clearing**  operation, however, consists of raytracing through a grid from the origin of the sensor outwards for each observation reported.

The marking and clearing operations can be defined in the  **obstacle layer**.

At this point, we can almost say that you already know how to configure both global and local costmaps. But if you remember, there's still a paramters file we haven't talked about. That's the  **common costmap parameters file**. These parameters will affect both the global and the local costmap.

Basically, the parameters you'll have to set in this file are the following:

-   **footprint**: Footprint is the contour of the mobile base. In ROS, it is represented by a two-dimensional array of the form [x0, y0], [x1, y1], [x2, y2], ...]. This footprint will be used to compute the radius of inscribed circles and circumscribed circles, which are used to inflate obstacles in a way that fits this robot. Usually, for safety, we want to have the footprint be slightly larger than the robot’s real contour.  
    
-   **robot_radius**: In case the robot is circular, we will specify this parameter instead of the footprint.  
    
-   **layers parameters**: Each layer has its own parameters:



### Obstacle Layer

The obstacle layer is in charge of the  **marking and clearing operations**.

As you already know, the costmap automatically subscribes to the sensor topics and updates itself according to the data it receives from them. Each sensor is used to either mark (insert obstacle information into the costmap), clear (remove obstacle information from the costmap), or both.

A marking operation is just an index into an array to change the cost of a cell.  
A clearing operation, however, consists of raytracing through a grid from the origin of the sensor outwards for each observation reported.

The marking and clearing operations can be defined in the obstacle layer.

-   **max_obstacle_height (default: 2.0)**: The maximum height of any obstacle to be inserted into the costmap, in meters. This parameter should be set to be slightly higher than the height of your robot.
-   **obstacle range (default: 2.5)**: The default maximum distance from the robot at which an obstacle will be inserted into the cost map, in meters. This can be overridden on a per-sensor basis.
-   **raytrace_range (default: 3.0)**: The default range in meters at which to raytrace out obstacles from the map using sensor data. This can be overridden on a per-sensor basis.
-   **observation_sources (default: "")**: A list of observation source names separated by spaces. This defines each of the  _source_name_  namespaces defined below.

Each **source_name** in observation_sources defines a namespace in which parameters can be set:

-   **/source_name/topic (default: source_name)**: The topic on which sensor data comes in for this source. Defaults to the name of the source.
-   **/source_name/data_type (default: "PointCloud")**: The data type associated with the topic, right now only "PointCloud," "PointCloud2," and "LaserScan" are supported.
-   **/source_name/clearing (default: false)**: Whether or not this observation should be used to clear out freespace.
-   **/source_name/marking (default: true)**: Whether or not this observation should be used to mark obstacles.
-   **/source_name/inf_is_valid (default: false)**: Allows for Inf values in "LaserScan" observation messages. The Inf values are converted to the laser's maximum range.

**VERY IMPORTANT:** A very important thing to keep in mind is that the  **obstacle layer**  uses different plugins for the  **local costmap**  and the  **global costmap**. For the local costmap, it uses the  **costmap_2d::ObstacleLayer**, and for the global costmap it uses the  **costmap_2d::VoxelLayer**. This is very important because it is a common error in Navigation to use the wrong plugin for the obstacle layers.

---
### Inflation Layer

The inflation layer is in charge of performing inflation in each cell with an obstacle.

-   **inflation_radius (default: 0.55)**: The radius in meters to which the map inflates obstacle cost values.
-   **cost_scaling_factor (default: 10.0)**: A scaling factor to apply to cost values during inflation.

### Static Layer

The static layer is in charge of providing the static map to the costmaps that require it (global costmap).

-   **map_topic (string, default: "map")**: The topic that the costmap subscribes to for the static map.

---

## Recovery Behaviors

It could happen that while trying to perform a trajectory, the robot gets stuck for some reason. Fortunately, if this happens, the ROS Navigation Stack provides methods that can help your robot to get unstuck and continue navigating. These are the  **recovery behaviors**.

The ROS Navigation Stack provides 2 recovery behaviors:  **clear costmap**  and  **rotate recovery**.

In order to enable the recovery behaviors, we need to set the following parameter in the move_base parameters file:

-   **recovery_behavior_enabled (default: true)**: Enables or disables the recovery behaviors.

### Rotate Recovery

Bascially, the rotate recovery behavior is a simple recovery behavior that attempts to clear out space by rotating the robot 360 degrees. This way, the robot may be able to find an obstacle-free path to continue navigating.

It has some parameters that you can customize in order to change or improve its behavior:

### Rotate Recovery Parameters

-   **/sim_granularity (double, default: 0.017)**: The distance, in radians, between checks for obstacles when checking if an in-place rotation is safe. Defaults to 1 degree.
-   **/frequency (double, default: 20.0)**: The frequency, in HZ, at which to send velocity commands to the mobile base.

### Other Parameters

IMPORTANT: These parameters are already set when using the base_local_planner local planner; they only need to be set explicitly for the recovery behavior if a different local planner is used.**

-   **/yaw_goal_tolerance (double, default: 0.05)**: The tolerance, in radians, for the controller in yaw/rotation when achieving its goal
-   **/acc_lim_th (double, default: 3.2)**: The rotational acceleration limit of the robot, in radians/sec^2
-   **/max_rotational_vel (double, default: 1.0)**: The maximum rotational velocity allowed for the base, in radians/sec
-   **/min_in_place_rotational_vel (double, default: 0.4)**: The minimum rotational velocity allowed for the base while performing in-place rotations, in radians/sec

### Clear Costmap

The clear costmap recovery is a simple recovery behavior that clears out space by clearing obstacles outside of a specified region from the robot's map. Basically, the local costmap reverts to the same state as the global costmap.

The radius away from the robot (in meters), in which obstacles will be removed from the costmap when they revert to the static map, can be setted by modifying the next parameter:

-   **/reset_distance**

This parameter is set in the move_base parameters file. Go back to the move_base section in order to refresh it.

The move_base node also provides a service in order to clear out obstacles from a costmap. This service is called  **/move_base/clear_costmaps**.

Bear in mind that by clearing obstacles from a costmap, you will make these obstacles invisible to the robot. So, be careful when calling this service since it could cause the robot to start hitting obstacles.

### Oscillation Supression

Oscillation occurs when, in any of the x, y, or theta dimensions, positive and negative values are chosen consecutively.

To prevent oscillations, when the robot moves in any direction, the opposite direction is marked invalid for the next cycle, until the robot has moved beyond a certain distance from the position where the flag was set.

In order to manage this issue, 2 parameters exist that you can set in the move_base parameters file.

-   **oscillation_timeout (double, default: 0.0)**: How long, in seconds, to allow for oscillation before executing recovery behaviors. A value of 0.0 corresponds to an infinite timeout.
-   **oscillation_distance (double, default: 0.5)**: How far, in meters, the robot must move to not be considered oscillating. Moving this far resets the timer counting up to the ~oscillation_timeout

---
---
---

## Recap

Congratulations! At this point, you've already seen almost all of the important parts that this chapter covers. And since this is the last chapter of the course, this means that you are very close to knowing how to deal with ROS Navigation in its entirety!

Anyways, you may be overwhelmed with all of the information that you've received about Path Planning. That's why I think this is a good moment to do a summary of all that you've seen in this chapter up until now. Let's begin!

### The move_base node[](https://i-0ac7cf2b39a77c204.robotigniteacademy.com/jupyter/notebooks/Path_Planning_2.ipynb#The-move_base-node)

The move_base node is, basically, the node that coordinates all of the Path Planning System. It takes a goal pose as input, and outputs the necessary velocity commands in order to move the robot from an initial pose to the specified goal pose. In order to achieve this, the move_base node manages a whole internal process where it take place for different parts:

-   global planner
-   local planner
-   costmaps
-   recovery behaviors

### The global planner[](https://i-0ac7cf2b39a77c204.robotigniteacademy.com/jupyter/notebooks/Path_Planning_2.ipynb#The-global-planner)

When a new goal is received by the move_base node, it is immediately sent to the global planner. The global planner, then, will calculate a safe path for the robot to use to arrive to the specified goal. The global planner uses the global costmap data in order to calculate this path.

There are different types of global planners. Depending on your setup, you will use one or another.

### The local planner[](https://i-0ac7cf2b39a77c204.robotigniteacademy.com/jupyter/notebooks/Path_Planning_2.ipynb#The-local-planner)

Once the global planner has calculated a path for the robot, this is sent to the local planner. The local planner, then, will execute this path, breaking it into smaller (local) parts. So, given a plan to follow and a map, the local planner will provide velocity commands in order to move the robot. The local planner operates over a local costmap.

There are different types of local planners. Depending on the kind of performance you require, you will use one or another.

### Costmaps[](https://i-0ac7cf2b39a77c204.robotigniteacademy.com/jupyter/notebooks/Path_Planning_2.ipynb#Costmaps)

Costmaps are, basically, maps that represent which points of the map are safe for the robot to be in, and which ones are not. There are 2 types of costmaps:

-   global costmap
-   local costmap

Basically, the difference between them is that the global costmap is built using the data from a previously built static map, while the local costmap is built from the robot's sensor readings.

### Recovery Behaviors[](https://i-0ac7cf2b39a77c204.robotigniteacademy.com/jupyter/notebooks/Path_Planning_2.ipynb#Recovery-Behaviors)

The recovery behaviors provide methods for the robot in case it gets stuck. The Navigation Stack provides 2 different recovery behaviors:

-   rotate recovery
-   clear costmap

### Configuration[](https://i-0ac7cf2b39a77c204.robotigniteacademy.com/jupyter/notebooks/Path_Planning_2.ipynb#Configuration)

Since there are lots of different nodes working together, the number of parameters available to configure the different nodes is also very high. I think it would be a great idea if we summarize the different parameter files that we will need to set for Path Planning. The parameter files you'll need are the following:

-   **move_base_params.yaml**
-   **global_planner_params.yaml**
-   **local_planner_params.yamls**
-   **common_costmap_params.yaml**
-   **global_costmap_params.yaml**
-   **local_costmap_params.yaml**

Besides the parameter files shown above, we will also need to have a launch file in order to launch the whole system and load the different parameters.

### Overall[](https://i-0ac7cf2b39a77c204.robotigniteacademy.com/jupyter/notebooks/Path_Planning_2.ipynb#Overall)

Summarizing, this is how the whole path planning method goes:

After getting the current position of the robot, we can send a goal position to the  **move_base**  node. This node will then send this goal position to a  **global planner**  which will plan a path from the current robot position to the goal position. This plan is in respect to the  **global costmap**, which is feeding from the  **map server**.

The  **global planner**  will then send this path to the  **local planner**, which executes each segment of the global plan. The  **local planner**  gets the odometry and the laser data values and finds a collision-free local plan for the robot. The  **local planner**  is associated with the  **local costmap**, which can monitor the obstacle(s) around the robot. The  **local planner**  generates the velocity commands and sends them to the base controller. The robot base controller will then convert these commands into real robot movement.

If the robot is stuck somewhere, the recovery behavior nodes, such as the  **clear costmap recovery**  or  **rotate recovery**, will be called.

Now everything makes more sense, right?

---

## Useful Rviz-ualizations

### Robot Footprint
![](https://raw.githubusercontent.com/rwbot/ros-nav-5days/master/footprint.png)

### Rainbow Cost Grid

> Hi @rwbot , to have the 'rainbow' map of your local planner, you'll have to set another parameter called `publish_cost_grid_pc: true`. Unfortunately, this parameter is not available on rqt_reconfigure, so you'll have to do it manually. Here's the general steps:
1. Create your own package,  say `my_husky`
2. Copy the config file `planner.yaml` from the original:
`$ roscd husky_navigation/config`
`$ cp planner.yaml /home/user/catkin_ws/my_husky/config`
3. Copy the launch files:
`$ roscd husky_navigation/launch`
`$ cp move_base_demo.launch /home/user/catkin_ws/my_husky/launch`
`$ cp move_base.launch /home/user/catkin_ws/my_husky/launch`
4. Open `planner.yaml`, under `DWAPlannerROS`, add `publish_cost_grid_pc: true` (say, at line 74)
5. Open `move_base.launch` (line 38), point rosparam to your package 
`<rosparam file="$(find my_husky)/config/planner.yaml" command="load"/>`
6. Open `move_base_demo.launch` (line 36), point it to your package 
`<include file="$(find my_husky)/launch/move_base.launch" />`
7. Finally execute `$ roslaunch my_husky move_base_demo.launch`
8. Then open RViz, and you should get something like this when you give Husky a goal

![](https://raw.githubusercontent.com/rwbot/ros-nav-5days/master/rainbow_cost_map.png)


/
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTE0NzA4NTY0NDNdfQ==
-->