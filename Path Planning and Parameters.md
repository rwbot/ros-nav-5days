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

Anyway, this is not the only topic that the move_base Action Server provides. As every action server, it provides the following 5 topics:

-   **move_base/goal (move_base_msgs/MoveBaseActionGoal)**
-   **move_base/cancel (actionlib_msgs/GoalID)**
-   **move_base/feedback (move_base_msgs/MoveBaseActionFeedback)**
-   **move_base/status (actionlib_msgs/GoalStatusArray)**
-   **move_base/result (move_base_msgs/MoveBaseActionResult)**









































#
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTE3ODY4OTc3NTQsLTU5Njc0NTg1MV19
-->