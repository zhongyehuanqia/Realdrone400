if [ -f "$HOME/real_drone_400/devel/setup.bash" ]; then
  source "$HOME/real_drone_400/devel/setup.bash"
elif [ -f "$HOME/桌面/REAL_DRONE_400-main/devel/setup.bash" ]; then
  source "$HOME/桌面/REAL_DRONE_400-main/devel/setup.bash"
else
  echo "找不到 devel/setup.bash"
  exit 1
fi
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
