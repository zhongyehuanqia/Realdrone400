if [ -f "$HOME/real_drone_400/devel/setup.bash" ]; then
  . "$HOME/real_drone_400/devel/setup.bash"
elif [ -f /home/reallab/real_drone_400/devel/setup.bash ]; then
  . /home/reallab/real_drone_400/devel/setup.bash
elif [ -f "$HOME/桌面/REAL_DRONE_400-main/devel/setup.bash" ]; then
  . "$HOME/桌面/REAL_DRONE_400-main/devel/setup.bash"
else
  echo "找不到 devel/setup.bash"
  exit 1
fi

sudo chmod 777 /dev/ttyACM0 & sleep 2;
roslaunch mavros px4.launch & sleep 3;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;

roslaunch livox_ros_driver2 msg_MID360.launch;
