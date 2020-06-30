# Pure Pursuit Controller for ROS
Pure Pursuit Path Tracking Controller for ROS (Modified from [here](https://github.com/jmaye/pure-pursuit-controller-ros)), tested on an Ackermann-steering vehicle.
![](images/rviz.png)

## Environment:
- Ubuntu 18.04
- ROS Melodic


## Requirements:
1. **tf from map to base_link**, which can be obtained using *[AMCL](http://wiki.ros.org/amcl)* node or *[robot_localization](http://docs.ros.org/melodic/api/robot_localization/html/index.html)* node.

2. The *reference_path* is published on *'map'* frame. 


## Usage:

### Configuration:

**See pure_pursuit_controller.yaml**

### Launch:
0. Make .py Executable
```bash 
    $chmod +x [path_to_your_workspace]/scripts/show_trajectory.py
    $chmod +x [path_to_your_workspace]/scripts/test_send_ref_path.py
```

1. Run the Pure Pursuit Controller:

```bash 
    $roslaunch pure_pursuit_controller pure_pursuit.launch
```

2. Send waypoints reference path

```bash
    $roslaunch pure_pursuit_controller test_send_waypoints.launch
```

### Tuning:
In General, the ***look_ahead_ratio*** and ***look_ahead_constant*** in *pure_pursuit_controller.yaml* determines the Look-ahead-Distance in Pure-Pursuit, which mainly affects the Path Tracking Performance. See [Matlab: PurePursuit](https://ww2.mathworks.cn/help/robotics/ug/pure-pursuit-controller.html) for more information.


## Reference:

- [Pure-Pursuit ROS](https://github.com/jmaye/pure-pursuit-controller-ros)

- [PythonRobotics: PurePursuit](https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/pure_pursuit)

- [Matlab: PurePursuit](https://ww2.mathworks.cn/help/robotics/ug/pure-pursuit-controller.html)