# Create the workspace
```
export STAR_WORKSPACE=~/star_ws/src/shared_ctrl
mkdir -p $STAR_WORKSPACE
cd $STAR_WORKSPACE
```

# Clone the repos
```
git clone git@github.com:simonleonard/ur5e_ign.git
git clone git@github.com:simonleonard/cartesian_controllers.git -b shared-ctrl
```

# Build the code
```
cd ~/star_ws
rosdep install --ignore-src --from-paths src/shared_ctrl
colcon build
```

# Run the simulation
```
source install/setup.bash
ros2 launch ur5e_ign ur5e_ign.launch.py
```

# Move the robot (Cartesian position)
You need to send a command to the topic /cartesian_motion_controller/target_frame. Try from the start position
```
ros2 topic pub -1 /cartesian_motion_controller/target_frame geometry_msgs/msg/PoseStamped '{header: { frame_id: "base_link"}, pose: {position: {x: -0.4917195715569225, y: 0.13329963419516075, z: 0.4}, orientation: {x: 0.7071065569246718, y: 0.7071065591331009, z: 0, w: 0} } }'
```

# Activate x coordinate for teleop, then press "i" or "," in the teleop terminal
```
ros2 param set /cartesian_motion_controller x true
```
