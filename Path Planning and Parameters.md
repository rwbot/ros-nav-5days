# Path Planning & Parameters


## The move_base package

The move_base package contains the  **move_base node**. Doesn't that sound familiar? Well, it should, since you were introduced to it in the Basic Concepts chapter! The move_base node is one of the major elements in the ROS Navigation Stack, since it links all of the elements that take place in the Navigation process. Let's say it's like the Architect in Matrix, or the Force in Star Wars. Without this node, the ROS Navigation Stack wouldn't make any sense!

Ok! We understand that the move_base node is very important, but... what is it exactly? What does it do? Great question!

The  **main function of the move_base node is to move the robot from its current position to a goal position**. Basically, this node is an implementation of a  **SimpleActionServer**, which takes a goal pose with message type  **geometry_msgs/PoseStamped**. Therefore, we can send position goals to this node by using a  **SimpleActionClient**.

This Action Server provides the topic  **move_base/goal**, which is the input of the Navigation Stack. This topic is then used to provide the goal pose.
<!--stackedit_data:
eyJoaXN0b3J5IjpbODU2MjEwNjEsLTU5Njc0NTg1MV19
-->