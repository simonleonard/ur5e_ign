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

# Activate x coordinate for teleop, then press "i" or "," in the teleop terminal
```
ros2 param set /cartesian_motion_controller x true
```
