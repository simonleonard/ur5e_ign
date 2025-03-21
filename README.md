# create the workspace
export STAR_WORKSPACE=~/star_ws/src/shared_ctrl
mkdir -p $STAR_WORKSPACE
cd $STAR_WORKSPACE

# clone the repos
git clone git@github.com:simonleonard/ur5e_ign.git
git clone git@github.com:simonleonard/cartesian_controllers.git -b shared-ctrl

# build the code
cd ~/star_ws
rosdep install --ignore-src --from-paths src/shared_ctrl
colcon build

# run the simulation
source install/setup.bash
ros2 launch ur5e_ign ur5e_ign.launch.py

# activate x coordinate for teleop
ros2 param set /cartesian_motion_controller x true
