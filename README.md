<<<<<<< HEAD
<<<<<<< HEAD
# gesture-controlled-dobot-arm
A ROS 2-based gesture control system for the Dobot Magician robotic arm using MediaPipe and OpenCV. Real-time joint control via hand tracking and finger gesture recognition.

## Getting Started
```bash
git clone https://github.com/Venkatesh7981/final_year_project
cd ros2_4dof_arm
source install/setup.bash

## Terminal 1

cd ~/gesture-controlled-dobot-arm_backup
rm -rf build/ install/ log/
colcon build --packages-select gesture_control_pkg --symlink-install
source install/setup.bash

ros2 run gesture_control_pkg angle_to_joint_state_publisher

## Terminal 2

cd /home/venkatesh/gesture-controlled-dobot-arm_backup
source install/setup.bash
ros2 run gesture_control_pkg gesture_angle_publisher

## Terminal 3

source install/setup.bash
 ros2 launch dobot_description display.launch.py


=======
# Final_Year_Project
>>>>>>> d4208bcad9025fc567104a4cfce0da91f7fc1327
=======

>>>>>>> b52cf20586ed9900c63bf14f708e410891c16d14
