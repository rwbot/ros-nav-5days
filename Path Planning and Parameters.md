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























#
<!--stackedit_data:
eyJoaXN0b3J5IjpbMzUyNTYxMjUwLDEyMDIxMzg3NDksLTMyMz
IxMjIyMywtMTIxMzM4ODM5NiwxODg3MTY1MjMxLC01OTY3NDU4
NTFdfQ==
-->