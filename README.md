# 流程
无人机检测地面数字0-9，按数字大小顺序依次降落在这些点上

# 安装和使用
放在catkin_ws/src/ 文件夹下
cd catkin_ws/build
make
需要ROS
roscore
rosrun ardrone_autonomy ardrone_driver
rosrun air_hockey drone_keyboard
rosrun air_hockey drone_vision
