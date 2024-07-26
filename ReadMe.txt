编译
colcon build
运行
source install/setup.bash 
ros2 launch pca9685_ros2_control_example joint_group_velocity_example.launch.py

测试
ros2 topic pub /joint_group_velocity_controller/commands std_msgs/msg/Float64MultiArray '{"data":[0.73, -0.21, 0.64, 0.35]}'


安装控制器
sudo dpkg -i ros-humble-mecanum-drive-controller_0.8.0-0jammy_arm64.deb

麦轮运行测试
ros2 launch pca9685_ros2_control_example mecanum_drive_example.launch.py
测试
ros2 run teleop_twist_keyboard teleop_twist_keyboard # 在调试笔记本电脑上启动手动遥控

命令测试
ros2 launch pca9685_ros2_control_example pid_example.launch.py
ros2 topic pub /pid_controller/reference control_msgs/msg/MultiDOFCommand '{"dof_names":["joint_1","joint_2","joint_3","joint_4"],"values":[7.0,7.0,7.0,7.0]}'


bzlrobot@raspberrypi:~$ ros2 node info /pid_controller 
/pid_controller
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /pid_controller/reference: control_msgs/msg/MultiDOFCommand
  Publishers:
    /gains/joint_1/pid_state: control_msgs/msg/PidState
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /pid_controller/controller_state: control_msgs/msg/MultiDOFStateStamped
    /pid_controller/transition_event: lifecycle_msgs/msg/TransitionEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /pid_controller/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /pid_controller/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /pid_controller/get_parameters: rcl_interfaces/srv/GetParameters
    /pid_controller/list_parameters: rcl_interfaces/srv/ListParameters
    /pid_controller/set_feedforward_control: std_srvs/srv/SetBool
    /pid_controller/set_parameters: rcl_interfaces/srv/SetParameters
    /pid_controller/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

bzlrobot@raspberrypi:~$ ros2 topic pub /pid_controller/reference control_msgs/msg/MultiDOFCommand '{"dof_names":["joint_1"],"values":[7.0]}'

bzlrobot@raspberrypi:~$ ros2 topic echo /pid_controller/reference
bzlrobot@raspberrypi:~$ ros2 topic echo /pid_controller/controller_state
A message was lost!!!
	total count change:1
	total count: 1---
header:
  stamp:
    sec: 1721699667
    nanosec: 39229305
  frame_id: ''
dof_states:
- name: joint_1
  reference: 6.0
  feedback: 6.073745796940267
  feedback_dot: -1067.092995672394
  error: -0.07374579694026728
  error_dot: .nan
  time_step: 0.112781741
  output: 0.112183812038508
---
header:
  stamp:
    sec: 1721699667
    nanosec: 144199842
  frame_id: ''
dof_states:
- name: joint_1
  reference: 6.0
  feedback: 6.073745796940267
  feedback_dot: -1066.460750150859
  error: -0.07374579694026728
  error_dot: .nan
  time_step: 0.104446778
  output: 0.11210678692969346
---

     static std::map<int, control_toolbox::Pid::Gains> pid_gains = 
  {
    {-9, { 0.025,    0.0235,    0.00210,   0.99,  0.0, false}},  
    {-8, { 0.025,    0.0235,    0.00210,   0.99,  0.0, false}},
    {-7, { 0.025,    0.0235,    0.00210,   0.99,  0.0, false}},
    {-6, { 0.025,    0.0150,    0.00215,   0.99,  0.0, false}},
    {-5, { 0.025,    0.0150,    0.00215,   0.99,  0.0, false}},
    {-4, { 0.023,    0.0060,    0.00100,   0.99,  0.0, false}},
    {-3, { 0.010,    0.0095,    0.00130,   0.99,  0.0, false}},
    {-2, { 0.007,    0.0060,    0.00100,   0.99,  0.0, false}},
    {-1, { 0.002,    0.0030,    0.00100,   0.99,  0.0, false}},

    {0, { 0.002,    0.0030,    0.00200,   0.0,  -0.99, false}},
    {1, { 0.002,    0.0030,    0.00100,   0.0,  -0.99, false}},
    {2, { 0.007,    0.0060,    0.00100,   0.0,  -0.99, false}},
    {3, { 0.010,    0.0095,    0.00130,   0.0,  -0.99, false}},
    {4, { 0.023,    0.0060,    0.00100,   0.0,  -0.99, false}},
    {5, { 0.025,    0.0150,    0.00215,   0.0,  -0.99, false}},
    {6, { 0.025,    0.0150,    0.00215,   0.0,  -0.99, false}},
    {7, { 0.028,    0.0235,    0.00210,   0.0,  -0.99, false}},
    {8, { 0.028,    0.0235,    0.00210,   0.0,  -0.99, false}},
    {9, { 0.028,    0.0235,    0.00210,   0.0,  -0.99, false}},  
    {10, { 0.028,    0.0235,    0.00210,   0.0,  -0.99, false}},    
    {11, { 0.028,    0.0235,    0.00210,   0.0,  -0.99, false}},   
    {12, { 0.028,    0.0235,    0.00210,   0.0,  -0.99, false}}    
 
  };

      static std::map<int, control_toolbox::Pid::Gains> pid_gains = 
  {
    {-9, { 0.025,    0.0235,    0.00210,   0.99,  -0.99, false}},  
    {-8, { 0.025,    0.0235,    0.00210,   0.99,  -0.99, false}},
    {-7, { 0.025,    0.0235,    0.00210,   0.99,  -0.99, false}},
    {-6, { 0.025,    0.0150,    0.00215,   0.99,  -0.99, false}},
    {-5, { 0.025,    0.0150,    0.00215,   0.99,  -0.99, false}},
    {-4, { 0.023,    0.0060,    0.00100,   0.99,  -0.99, false}},
    {-3, { 0.010,    0.0095,    0.00130,   0.99,  -0.99, false}},
    {-2, { 0.007,    0.0060,    0.00100,   0.99,  -0.99, false}},
    {-1, { 0.002,    0.0030,    0.00100,   0.99,  -0.99, false}},

    {0, { 0.002,    0.0030,    0.00200,   0.99,  -0.99, false}},
    {1, { 0.002,    0.0030,    0.00100,   0.99,  -0.99, false}},
    {2, { 0.007,    0.0060,    0.00100,   0.99,  -0.99, false}},
    {3, { 0.010,    0.0095,    0.00130,   0.99,  -0.99, false}},
    {4, { 0.023,    0.0060,    0.00100,   0.99,  -0.99, false}},
    {5, { 0.025,    0.0150,    0.00215,   0.99,  -0.99, false}},
    {6, { 0.025,    0.0150,    0.00215,   0.99,  -0.99, false}},
    {7, { 0.028,    0.0235,    0.00210,   0.99,  -0.99, false}},
    {8, { 0.028,    0.0235,    0.00210,   0.99,  -0.99, false}},
    {9, { 0.028,    0.0235,    0.00210,   0.99,  -0.99, false}},  
    {10, { 0.028,    0.0235,    0.00210,   0.99,  -0.99, false}},    
    {11, { 0.028,    0.0235,    0.00210,   0.99,  -0.99, false}},   
    {12, { 0.028,    0.0235,    0.00210,   0.99,  -0.99, false}}    
 
  };
