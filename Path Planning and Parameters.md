# Path Planning & Parameters


## The move_base package

The move_base package contains the  **move_base node**. Doesn't that sound familiar? Well, it should, since you were introduced to it in the Basic Concepts chapter! The move_base node is one of the major elements in the ROS Navigation Stack, since it links all of the elements that take place in the Navigation process. Let's say it's like the Architect in Matrix, or the Force in Star Wars. Without this node, the ROS Navigation Stack wouldn't make any sense!

Ok! We understand that the move_base node is very important, but... what is it exactly? What does it do? Great question!

The  **main function of the move_base node is to move the robot from its current position to a goal position**. Basically, this node is an implementation of a  **SimpleActionServer**, which takes a goal pose with message type  **geometry_msgs/PoseStamped**. Therefore, we can send position goals to this node by using a  **SimpleActionClient**.

This Action Server provides the topic  **move_base/goal**, which is the input of the Navigation Stack. This topic is then used to provide the goal pose.

So, each time you set a Pose Goal using the 2D Nav Goal tool from RViz, what is really happening is that a new message is being published into the move_base/goal topic.

```yaml
user:~$ rostopic echo -n1 /move_base/goal

header:
  seq: 4
  stamp:
    secs: 1417
    nsecs: 306000000
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs:         0
  id: ''
goal:
  target_pose:
    header:
      seq: 4
      stamp:
        secs: 1417
        nsecs: 306000000
      frame_id: "map"
    pose:
      position:
        x: -2.50387191772
        y: 0.378415107727
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: -0.552616699648
        w: 0.833435530362
---
```

This is not the only topic that the move_base Action Server provides. As every action server, it provides the following 5 topics:

-   **move_base/goal (move_base_msgs/MoveBaseActionGoal)**
-   **move_base/cancel (actionlib_msgs/GoalID)**
-   **move_base/feedback (move_base_msgs/MoveBaseActionFeedback)**
-   **move_base/status (actionlib_msgs/GoalStatusArray)**
-   **move_base/result (move_base_msgs/MoveBaseActionResult)**

##### Using this Action Client, move the robot to three different Poses of the Map. When the robot has reached the 3 poses, start over again creating a loop, so that the robot will keep going to these 3 poses over and over.

* **send_goal_client_triangle.py**
```python
#! /usr/bin/env python
import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    
    print('[Feedback] Going to Goal Pose...')

# initializes the action client node
rospy.init_node('move_base_action_client')

# create the connection to the action server
client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
# waits until the action server is up and running
client.wait_for_server()

goal1 = MoveBaseGoal()
goal1.target_pose.header.frame_id = 'map'
goal1.target_pose.pose.position.x = -0.0551781728864
goal1.target_pose.pose.position.y = 1.04829895496
goal1.target_pose.pose.position.z = 0.0
goal1.target_pose.pose.orientation.x = 0.0
goal1.target_pose.pose.orientation.y = 0.0
goal1.target_pose.pose.orientation.z = -0.708673676941
goal1.target_pose.pose.orientation.w = 0.705536405589

goal2 = MoveBaseGoal()
goal2.target_pose.header.frame_id = 'map'
goal2.target_pose.pose.position.x = 1.98187232018
goal2.target_pose.pose.position.y = -3.01903486252
goal2.target_pose.pose.position.z = 0.0
goal2.target_pose.pose.orientation.x = 0.0
goal2.target_pose.pose.orientation.y = 0.0
goal2.target_pose.pose.orientation.z = 0.944016792756
goal2.target_pose.pose.orientation.w = 0.32989740071

goal3 = MoveBaseGoal()
goal3.target_pose.header.frame_id = 'map'
goal3.target_pose.pose.position.x = -2.04235863686
goal3.target_pose.pose.position.y = -2.94409298897
goal3.target_pose.pose.position.z = 0.0
goal3.target_pose.pose.orientation.x = 0.0
goal3.target_pose.pose.orientation.y = 0.0
goal3.target_pose.pose.orientation.z = 0.331192763356
goal3.target_pose.pose.orientation.w = 0.943563115801

while not rospy.is_shutdown():
    # creates a goal to send to the action server
    # sends the goal to the action server, specifying which feedback function
    # to call when feedback received
    client.send_goal(goal1, feedback_cb=feedback_callback)
    client.wait_for_result()
    print('[Result] State: %d'%(client.get_state()))
    client.send_goal(goal2, feedback_cb=feedback_callback)
    client.wait_for_result()
    print('[Result] State: %d'%(client.get_state()))
    client.send_goal(goal3, feedback_cb=feedback_callback)
    client.wait_for_result()
    print('[Result] State: %d'%(client.get_state()))

    

# Uncomment these lines to test goal preemption:
#time.sleep(3.0)
#client.cancel_goal()  # would cancel the goal 3 seconds after starting

# wait until the result is obtained
# you can do other stuff here instead of waiting
# and check for status from time to time 
# status = client.get_state()
# check the client API link below for more info
```

So, at this point, you've checked that you can send pose goals to the move_base node by sending messages to the /move_base/goal topic of its Action Server.

When this node  **receives a goal pose**, it links to components such as the  _global planner, local planner, recovery behaviors, and costmaps_, and  **generates an output, which is a velocity command**  with the message type  _geometry_msgs/Twist_, and sends it to the  **/cmd_vel**topic in order to move the robot.

The move_base node, just as you saw with the slam_gmapping and the amcl nodes in previous chapters, also has parameters that you can modify. For instance, one of the parameters that you can modify is the frequency at which the move_base node sends these velocity commands to the base controller.

---

# The Global Planner

When a new goal is received by the move_base node, this goal is immediately sent to the global planner. Then, the  **global planner is in charge of calculating a safe path in order to arrive at that goal pose**. This path is calculated before the robot starts moving, so it will  **not take into account the readings that the robot sensors are doing** while moving. Each time a new path is planned by the global planner, this path is published into the  **/plan**  topic.

When you send a goal in order to visualize the path plan made by the global planner, the robot automatically starts executing this plan. This happens because by sending this goal pose, you're starting the whole navigation process.

In some cases, you might be interested in just visualizing the global plan, but not in executing that plan. For this case, the move_base node provides a service named  **/make_plan**. This service allows you to **calculate a global plan without causing the robot to execute the path**.  The type of message used by the **/make_plan** service is **nav_msgs/GetPlan**. 

```
rosservice call /move_base/make_plan TABTAB
```
When filing this message in order to call the service, you don't have to fill all of the fields of the message
```yaml
rosservice call /move_base/make_plan 
"start:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose:
    position:
      x: 1.16
      y: -4.76
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.75
      w: 0.69
goal:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose:
    position:
      x: 1.16
      y: -4.50
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.75
      w: 0.69
tolerance: 0.0"
```

* **Returned Plan**

```yaml
plan:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs:         0
    frame_id: ''
  poses:
    -
      header:
        seq: 0
        stamp:
          secs: 276
          nsecs: 936000000
        frame_id: "map"
      pose:
        position:
          x: 1.15000024661
          y: -4.79999986589
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    -
      header:
        seq: 0
        stamp:
          secs: 276
          nsecs: 936000000
        frame_id: "map"
      pose:
        position:
          x: 1.15000024661
          y: -4.74999986514
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    -
      header:
        seq: 0
        stamp:
          secs: 276
          nsecs: 936000000
        frame_id: "map"
      pose:
        position:
          x: 1.15000024661
          y: -4.72499986477
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    -
      header:
        seq: 0
        stamp:
          secs: 276
          nsecs: 936000000
        frame_id: "map"
      pose:
        position:
          x: 1.15000024661
          y: -4.6999998644
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    -
      header:
        seq: 0
        stamp:
          secs: 276
          nsecs: 936000000
        frame_id: "map"
      pose:
        position:
          x: 1.15000024661
          y: -4.67499986403
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    -
      header:
        seq: 0
        stamp:
          secs: 276
          nsecs: 936000000
        frame_id: "map"
      pose:
        position:
          x: 1.15000024661
          y: -4.64999986365
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    -
      header:
        seq: 0
        stamp:
          secs: 276
          nsecs: 936000000
        frame_id: "map"
      pose:
        position:
          x: 1.15000024661
          y: -4.59999986291
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    -
      header:
        seq: 0
        stamp:
          secs: 276
          nsecs: 936000000
        frame_id: "map"
      pose:
        position:
          x: 1.15000024661
          y: -4.54999986216
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    -
      header:
        seq: 0
        stamp:
          secs: 276
          nsecs: 936000000
        frame_id: "map"
      pose:
        position:
          x: 1.16000000149
          y: -4.49999999851
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.75
          w: 0.69
```

---

So, you now know that the first step of this navigation process is to calculate a safe plan so that your robot can arrive to the user-specified goal pose. But... how is this path calculated?

There exist different global planners. Depending on your setup (the robot you use, the environment it navigates, etc.), you would use one or another. Let's have a look at the most important ones.

### Navfn
The Navfn planner is probably the most commonly used global planner for ROS Navigation. It uses Dijkstra's algorithm in order to calculate the shortest path between the initial pose and the goal pose. Below, you can see an animation of how this algorithm works.

# DJISKTRA GIF

### Carrot Planner

The carrot planner takes the goal pose and checks if this goal is in an obstacle. Then, if it is in an obstacle, it walks back along the vector between the goal and the robot until a goal point that is not in an obstacle is found. It, then, passes this goal point on as a plan to a local planner or controller. Therefore, this planner does not do any global path planning. It is helpful if you require your robot to move close to the given goal, even if the goal is unreachable. In complicated indoor environments, this planner is not very practical.  
  
This algorithm can be useful if, for instance, you want your robot to move as close as possible to an obstacle (a table, for instance).

### Global Planner

The global planner is a more flexible replacement for the navfn planner. It allows you to change the algorithm used by navfn (Dijkstra's algorithm) to calculate paths for other algorithms. These options include support for A∗, toggling quadratic approximation, and toggling grid path.

### Change the Global Planner

The global planner used by the move_base node it's usually specified in the move_base parameters file. It can be changed under one of these parameter definitions either in the launch or the parameter files
	
```yaml 
 base_global_planner: "navfn/NavfnROS" # Sets the Navfn Planner
base_global_planner: "carrot_planner/CarrotPlanner" # Sets the Carrot Planner
base_global_planner: "global_planner/GlobalPlanner" # Sets the Global Planner
```

The global planner also has its own parameters in order to customize its behaviour. The parameters for the global planner are also located in a YAML file. Depending on which global planner you use, the parameters to set will be different. In this course, we will have a look at the parameters for the navfn planner because it's the one that is most commonly used. If you are interested in seeing the parameters you can set for the other planners, you can have a look at them here:

carrot planner:  [http://wiki.ros.org/carrot_planner](http://wiki.ros.org/carrot_planner)

global planner:  [http://wiki.ros.org/global_planner](http://wiki.ros.org/global_planner)



### Navfn Parameters

-   **/allow_unknown (default: true)**: Specifies whether or not to allow navfn to create plans that traverse unknown space. NOTE: if you are using a layered costmap_2d costmap with a voxel or obstacle layer, you must also set the track_unknown_space param for that layer to be true, or it will convert all of your unknown space to free space (which navfn will then happily go right through).
-   **/planner_window_x (default: 0.0)**: Specifies the x size of an optional window to restrict the planner to. This can be useful for restricting NavFn to work in a small window of a large costmap.
-   **/planner_window_y (default: 0.0)**: Specifies the y size of an optional window to restrict the planner to. This can be useful for restricting NavFn to work in a small window of a large costmap.
-   **/default_tolerance (default: 0.0)**: A tolerance on the goal point for the planner. NavFn will attempt to create a plan that is as close to the specified goal as possible, but no farther away than the default_tolerance.

-   **cost_factor**
-   **neutral_cost**
-   **lethal_cost**

Here you can see an example of a global planner parameters file:
```yaml
NavfnROS:
  visualize_potential: false    
  allow_unknown: false          
                                
  planner_window_x: 0.0         
  planner_window_y: 0.0         

  default_tolerance: 0.0
```

---

# Costmap
A costmap is a map that represents places that are safe for the robot to be in a grid of cells. Usually, the values in the costmap are binary, representing either free space or places where the robot would be in collision.

Each cell in a costmap has an integer value in the range {0,255}. There are some special values frequently used in this range, which work as follows:

-   **255 (NO_INFORMATION)**: Reserved for cells where not enough information is known.
-   **254 (LETHAL_OBSTACLE**: Indicates that a collision-causing obstacle was sensed in this cell
-   **253 (INSCRIBED_INFLATED_OBSTACLE)**: Indicates no obstacle, but moving the center of the robot to this location will result in a collision
-   **0 (FREE_SPACE)**: Cells where there are no obstacles and, therefore, moving the center of the robot to this position will not result in a collision

There exist 2 types of costmaps:  **global costmap**  and  **local costmap**. The main difference between them is, basically, the way they are built:

-   The  **global costmap**  is created from a static map.
-   The  **local costmap**  is created from the robot's sensor readings.

For now, we'll focus on the global costmap since it is the one used by the global planner. So,  **the global planner uses the global costmap in order to calculate the path to follow**.


## Global Costmap

The global costmap is created from a user-generated static map (as the one we created in the Mapping Chapter). In this case, the costmap is initialized to match the width, height, and obstacle information provided by the static map. This configuration is normally used in conjunction with a localization system, such as amcl. This is the method you'll use to initialize a  **global costmap**.  
  
The global costmap also has its own parameters, which are defined in a YAML file. Next, you can see an example of a global costmap parameters file.
```yaml
global_frame: map
static_map: true
rolling_window: false

plugins:
  - {name: static,                  type: "costmap_2d::StaticLayer"}
  - {name: inflation,               type: "costmap_2d::InflationLayer"}
  - {name: obstacles,               type: "costmap_2d::VoxelLayer"}
```

Costmap parameters are defined in 3 different files:

-   **global_costmap_params.yaml** : A YAML file that sets the parameters for the global costmap (which is the one you've seen above). 
-   **local_costmap_params.yaml** : A YAML file that sets the parameters for the local costmap.
-   **common_costmap_params.yaml** : A YAML file that sets the parameters for both the global and local costmaps. 

Fow now, we'll focus on the global costmap parameters since it's the costmap that is used by the global planner.

### Global Costmap Parameters[](https://i-08d82f91c773afd30.robotigniteacademy.com/jupyter/notebooks/Path_Planning_1.ipynb#Global-Costmap-Parameters)

The parameters you need to know are the following:

-   **global_frame (default: "/map")**: The global frame for the costmap to operate in.
-   **static_map (default: true)**: Whether or not to use a static map to initialize the costmap.
-   **rolling_window (default: false)**: Whether or not to use a rolling window version of the costmap. 

### **If the static_map parameter is set to true, this parameter must be set to false.**

-   **plugins**: Sequence of plugin specifications, one per layer. Each specification is a dictionary with a  **name**  and  **type**  fields. The name is used to define the parameter namespace for the plugin. This name will then be defined in the  **common_costmap_parameters.yaml**  file, which you will see in the the next Unit. The type field actually defines the plugin (source code) that is going to be used.

So, by setting the static_map parameter to true, and the rolling_window parameters to false, we will initialize the costmap by getting the data from a static map. This is the way you want to initialize a global costmap.






























#
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTIwNDEwMTM0OTcsMzUyNTYxMjUwLDEyMD
IxMzg3NDksLTMyMzIxMjIyMywtMTIxMzM4ODM5NiwxODg3MTY1
MjMxLC01OTY3NDU4NTFdfQ==
-->