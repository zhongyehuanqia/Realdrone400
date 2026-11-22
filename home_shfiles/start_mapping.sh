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

roslaunch fast_lio mapping_mid360.launch rviz:=false
