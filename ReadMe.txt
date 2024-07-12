编译
colcon build
运行
source install/setup.bash 
ros2 launch pca9685_ros2_control_example joint_group_velocity_example.launch.py

测试
ros2 topic pub /joint_group_velocity_controller/commands std_msgs/msg/Float64MultiArray '{"data":[0.73, -0.21, 0.64, 0.35]}'
