编译
colcon build
运行
source install/setup.bash 
ros2 launch pca9685_ros2_control_example joint_group_velocity_example.launch.py

测试
ros2 topic pub /joint_group_velocity_controller/commands std_msgs/msg/Float64MultiArray '{"data":[0.73, -0.21, 0.64, 0.35]}'


安装控制器
sudo dpkg -i ros-humble-mecanum-drive-controller_0.8.0-0jammy_arm64.deb
运行
ros2 launch pca9685_ros2_control_example mecanum_drive_example.launch.py
测试
ros2 run teleop_twist_keyboard teleop_twist_keyboard # 在调试笔记本电脑上启动手动遥控

