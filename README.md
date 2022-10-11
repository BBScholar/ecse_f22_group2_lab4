# ecse_f22_group2_lab4

The smart teleop node ensures that user input velocity commands are safe to run on the robot.

``` bash
# to run only the smart_teleop node
roslaunch smart_teleop smart_teleop.launch &

# to run only the smart_teleop node with namespace
roslaunch smart_teleop smart_teleop.launch use_namespace:=true namespace:=<whatever> &

# To run the smart_teleop node with the 2d simulator:
roslaunch smart_teleop launch_simulator.launch &
```
