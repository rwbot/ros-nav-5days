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