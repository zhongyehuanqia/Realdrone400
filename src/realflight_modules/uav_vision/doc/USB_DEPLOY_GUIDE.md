# U 盘版视觉包 — 香橙派部署步骤

这个包已经配好,拷到 U 盘插到香橙派,就能跑视觉检测。**不用网络、不用 scp**。

## 准备工作(在你 Windows 电脑上)

1. 找一个 U 盘(FAT32 格式,一般 U 盘默认就是),插到电脑。
2. 把 `uav_vision_usb` 文件夹里的 **`uav_vision`** 整个文件夹拷到 U 盘根目录。
3. 安全弹出 U 盘。

## 在香橙派上(串口终端操作)

### 第 1 步:插 U 盘,找到它

把 U 盘插到香橙派的 USB 口,在终端输入:

```bash
ls /media/orangepi/
```

 normally 会显示 U 盘名字(比如 `8A3E-1234` 或 U 盘名)。记下这个名字。

如果这一行是空的,再试:

```bash
df -h | grep media
```

找到类似 `/media/orangepi/XXXX` 的路径,那就是 U 盘挂载点。

### 第 2 步:拷贝包到主目录

假设 U 盘挂载在 `/media/orangepi/8A3E-1234`(把 `8A3E-1234` 换成你实际看到的):

```bash
cp -r /media/orangepi/8A3E-1234/uav_vision ~/uav_vision
```

然后确认拷过来了:

```bash
ls ~/uav_vision
```

应该看到 `CMakeLists.txt  config  images  launch  package.xml  scripts`。

> ⚠️ 这里拷到主目录 `~/uav_vision`,**不碰队友的 exploration_ws**。

### 第 3 步:建工作空间并编译

```bash
mkdir -p ~/catkin_ws/src
mv ~/uav_vision ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

`catkin_make` 要跑一两分钟,最后出现 `100% Built target` 且没有红色 error 就是成功。

### 第 4 步:测试(不用相机!)

编译成功后,先跑**离线测试**——包里自带一张赛题同款黑色双圆环测试图,程序循环发布它,检测节点实时识别:

**终端 A**(先开 roscore):
```bash
source ~/catkin_ws/devel/setup.bash
roscore
```

**终端 B**(再开检测):
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch uav_vision test_offline.launch
```

应该看到日志:`uav_vision: pattern=black, waiting for /camera/image_raw` 和 `发布测试图 ...`。

**终端 C**(看检测结果数值):
```bash
source ~/catkin_ws/devel/setup.bash
rostopic echo /vision/target_info
```

正常会看到检测到的中心坐标、半径等信息。看到数据在滚动,就说明**整条 ROS 检测链路跑通了**——这是视觉任务的核心里程碑。

### 第 5 步:看可视化(可选)

如果想看标注图像,在你 Windows 电脑上用 rqt_image_view 连板子比较麻烦(网络不通)。可以在板子上装个图形工具,或先只看第 4 步的 `target_info` 数值即可。

## 常见问题

| 现象 | 原因 / 解决 |
|---|---|
| `ls /media/orangepi/` 空的 | U 盘没识别:换 USB 口、重新插;或 `df -h` 找挂载点 |
| `catkin_make` 报找不到 cv_bridge | `sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport` |
| 编译权限报错 | 脚本没执行权限:`chmod +x ~/catkin_ws/src/uav_vision/scripts/*.py` 再 catkin_make |
| `roslaunch` 提示找不到包 | 没 source:`source ~/catkin_ws/devel/setup.bash`,每条新终端都要敲 |
| `target_info` 没有数据 | 确认 roscore 在跑、test_offline.launch 正常、测试图存在 |

## 关于真实相机

等离线测试跑通、拿到相机后,把第 4 步的 launch 换成:

```bash
roslaunch uav_vision test_camera.launch
```

它会启动 usb_cam(默认 /dev/video0)+ 检测节点。届时再根据实际画面调 `config/vision.yaml` 的阈值参数。

## 下一步目标

离线链路跑通 → 接真实相机调阈值 → 用 rosbag 录数据离线调参 → 把检测结果接入飞控(这部分等飞控队友的接口)。你现在只做视觉,先把第 4 步跑通就是胜利。
