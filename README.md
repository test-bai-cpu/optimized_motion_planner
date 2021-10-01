# Optimized Motion Planner
The optimized motion planner is developed based on RRTX motion planning 
algorithm and the CHOMP trajectory optimizer, referencing RRTXstatic class in OMPL library, Julia code in RRTX paper, and CHOMP planner code in MoveIt.

Compared with RRTX algorithm, optimizer is integrated. Compared with CHOMP algorithm, cost function of dynamic obstacles is developed.

The main algorithm is in `motion_planning/RRTX.cpp`. It subscribes from `optimized_motion_planner/obstacle_info_topic` to get information of obstacles. The message type of obstacle is in `msg/obstacle_info.msg`. The method of getting environments update can be modified according to user's requirements.

## Prerequisites

**ROS**: The ROS version is Noetic Ninjemys. The installation instruction is [here](http://wiki.ros.org/noetic/Installation).

## Example
One example is given in `src/example/start_example.cpp`. 
```bash
roslaunch optimized_motion_planner start_example.launch
```

## References

- <a name="RRTX"></a>Otte M, Frazzoli E. RRTX: Asymptotically optimal single-query sampling-based motion planning with quick replanning. The International Journal of Robotics Research. 2016;35(7):797-822.
- <a name="CHOMP"></a>Matthew Zucker, Nathan Ratliff, Anca Dragan, Mikhail Pivtoraiko, Matthew Klingensmith, Christopher Dellin, J. Andrew (Drew) Bagnell and Siddhartha Srinivasa Journal Article, International Journal of Robotics Research, Vol. 32, No. 9, pp. 1164-1193, Aug. 2013
- <a name="OMPL"></a>Ioan A. Șucan, Mark Moll, Lydia E. Kavraki, The Open Motion Planning Library, IEEE Robotics & Automation Magazine, vol. 19, no. 4, pp. 72-82, Dec. 2012.
- <a name="MoveIt"></a>David Coleman, Ioan A. Șucan, Sachin Chitta, Nikolaus Correll, Reducing the Barrier to Entry of Complex Robotic Software: a MoveIt! Case Study, Journal of Software Engineering for Robotics, 5(1):3–16, May 2014.