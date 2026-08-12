cd /root/ros2_ws/
source install/setup.bash
ros2 launch origincar_base origincar_bringup.launch.py #启动底盘

cd /root/ros2_ws/
source install/setup.bash
ros2 launch lslidar_driver lsn10_launch.py #启动雷达

cd /root/ros2_ws/
source install/setup.bash
ros2 launch racecar Run_nav.launch.py  #启动nav2导航节点

rviz2 #打开rviz2

cd /root/ros2_ws/src/racecar/scripts/main/
python3 saoma.py   #启动二维码文件

cd /root/ros2_ws/src/racecar/scripts/main/
python3 ai_qw.py   #启动图生文

source /opt/tros/humble/setup.bash
ros2 run hobot_tts hobot_tts --ros-args -p playback_device:="hw:1,0"  #启动文本转语音

amixer -c 1 sset 'DAC' 100%
amixer -c 1 sset 'HPL' 100%
amixer -c 1 sset 'HPR' 100%   #提高扬声器音量

cd /root/ros2_ws/src/racecar/scripts/src
python3 h1.py  #启动导航文件



cd /root/ros2_ws/
colcon build
source install/setup.bash   # 如果修改了nav.yaml完参数需要重新编译