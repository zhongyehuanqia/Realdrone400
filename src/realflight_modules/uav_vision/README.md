# uav_vision — 赛题视觉检测包

D 题「陆空协同无人机系统」的视觉部分：识别地面平台的**黑色双圆环 + 十字**图案，
输出相对位置误差与降落建议给飞控。**只做检测，不直接控制飞机。**

- 运行环境：Ubuntu 20.04 + ROS1 Noetic，机载 Orange Pi CM5（RK3588S）
- 相机：CAM-IM-202V，UVC 免驱，视频节点 `/dev/video20`，640×480
- 状态：离线图与真实相机均已验证，输出 `valid=True` 与归一化误差

## 目录

```
uav_vision/
├── CMakeLists.txt
├── package.xml
├── config/vision.yaml          # 检测参数（阈值、稳定帧、降落阈值）
├── launch/
│   ├── test_offline.launch     # 用自带测试图跑，不接相机
│   ├── test_camera.launch      # usb_cam 驱动方案（可选）
│   └── test_realcamera.launch  # 免驱动，直接读 /dev/video20（推荐）
├── images/test_pattern.png     # 赛题同款测试图
├── scripts/
│   ├── camera_publisher.py     # 读相机 → 发布 /camera/image_raw
│   ├── vision_node.py          # 核心检测：阈值+连通域 → 误差/降落建议
│   ├── image_publisher.py      # 离线测试图发布
│   ├── info_logger.py          # 把结果打印到终端（单串口调试用）
│   └── ground_station.py       # 地面站显示（需图形界面）
└── doc/
    ├── VISION_FLIGHT_INTERFACE.md  # 视觉—飞控接口约定（对接必读）
    └── USB_DEPLOY_GUIDE.md         # 香橙派部署步骤（串口/U盘）
```

## 快速开始

```bash
cd ~/catkin_ws && catkin_make && source devel/setup.bash
roslaunch uav_vision test_realcamera.launch
```

终端持续输出：

```
[RESULT] valid=True center=(320.1, 240.2) r=141.0 error=(0.001, -0.002) stable=15 landing=False
```

其中 `error=(ex, ey)` 是归一化位置误差（−1~1），交给飞控。

## 关键话题

| 方向 | 话题 | 类型 |
|---|---|---|
| 飞控 → 视觉 | `/mission/state` | `std_msgs/String`（IDLE/SEARCH/FOLLOW/DROP/LAND） |
| 视觉 → 飞控 | `/vision/target_error` | `geometry_msgs/PointStamped`（x=ex, y=ey, z=置信度） |
| 视觉 → 飞控 | `/vision/landing_recommendation` | `std_msgs/Bool` |

> 完整接口、符号约定、降落触发条件见 `doc/VISION_FLIGHT_INTERFACE.md`。

## 注意

- 检测脚本依赖 `cv2` 与 `cv_bridge`，且 `import cv2` 必须在 `from cv_bridge import ...` **之前**。
- `/mission/state` 由本机的 `vision_servo` 发布。只开 `vision_node`、不开发布者时，状态保持 `IDLE`，降落建议不会变成 true。
- 脚本需可执行权限：`chmod +x scripts/*.py`。

## 接到这架飞机

检测仍只看图。`vision_servo.py` 订阅 `/Odom_high_freq` 和 `/vision/target_error`，把画面误差变成 `/position_cmd`，并在视觉允许降落时停掉位置指令、再发 `/px4ctrl/takeoff_land`。

姿态用的是已经去掉雷达 15 度安装角的机体系，所以水平速度沿机头和机尾，不沿雷达斜轴。

下视相机默认画面上方对着机头：目标在画面右侧就往右飞，在画面下方就往机尾飞。方向反了改 `config/servo.yaml` 里的 `sign_ex`、`sign_ey`。

起飞悬停后执行 `home_shfiles/start_vision.sh`。6 通道保持在轨迹跟踪用的指令模式。不要同时开 `start_planner.sh`，两边都会发 `/position_cmd`。
