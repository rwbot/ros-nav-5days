# udacity_bot

ROS Nav Cheatsheet

### Find all parameters through echoing their /topic/parameter_descriptions and /topic/parameter_updates. Useful when you can't find the param definition in ROS wiki

```
rostopic echo -n1 /move_base/parameter_descriptions > move_base_param_desc.txt
rostopic echo -n1 /move_base/parameter_updates > move_base_param_updates.txt
```

Change ROSPARAMS while running!!!
## dynamic_reconfigure and its gui counterpart >> rqt_reconfigure 
http://wiki.ros.org/dynamic_reconfigure
```
rosrun dynamic_reconfigure dynparam COMMAND
```
http://wiki.ros.org/rqt_reconfigure
```
rosrun rqt_reconfigure rqt_reconfigure
```


## USE rqt_gui to launch rqt_reconfigure, like rviz, settings are saved as a .perspective file. Figure out how to launch with .perspective

```
rosrun rqt_gui rqt_gui
```

