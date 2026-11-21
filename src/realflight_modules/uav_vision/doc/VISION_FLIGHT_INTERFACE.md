# 视觉 — 飞控 接口约定（D 题陆空协同）

> 视觉侧：`uav_vision` 包，运行在香橙派（ROS1 Noetic / Ubuntu 20.04）
> 用途：视觉把"目标在哪、能不能降落"告诉飞控；飞控把"现在处于什么任务阶段"告诉视觉。
> **这份文档给飞控队友看，两边确认后签字，避免各写各的接不上。**

---

## 0. 一句话_summary

视觉只负责"看"和"算"，**不直接控制飞机**。它把误差以数字形式发布到话题上，飞控订阅后自己决定怎么飞。
唯一需要飞控回传给视觉的，是当前的**任务状态**（SEARCH / FOLLOW / DROP / LAND）。

---

## 1. 话题总表

| 方向 | 话题名 | 消息类型 | 谁发 | 内容 |
|---|---|---|---|---|
| 视觉订阅 | `/camera/image_raw` | `sensor_msgs/Image` | 相机节点 | 原始图像（当前 640×480） |
| **飞控 → 视觉** | `/mission/state` | `std_msgs/String` | **飞控** | 任务状态：`IDLE` / `SEARCH` / `FOLLOW` / `DROP` / `LAND` |
| **视觉 → 飞控** | `/vision/target_error` | `geometry_msgs/PointStamped` | 视觉 | 位置误差（最重要，见第 2 节） |
| 视觉 → 飞控 | `/vision/landing_recommendation` | `std_msgs/Bool` | 视觉 | 是否可以执行降落，`true` = 满足降落条件 |
| 视觉 → 飞控 | `/vision/target_info` | `std_msgs/String` | 视觉 | 完整 JSON 结果（调试/地面站用，见第 5 节） |
| 视觉 → 地面站 | `/vision/overlay/image_raw` | `sensor_msgs/Image` | 视觉 | 画了圆圈和十字的标注图 |
| 视觉调试 | `/vision/debug_mask` | `sensor_msgs/Image` | 视觉 | 二值化掩膜，仅 `debug: true` 时发布 |

> ⚠️ **关键提醒**：`/mission/state` 必须由飞控（或地面站）发布。视觉代码只是**接收**它。
> 如果没人发，状态永远是 `IDLE`，`landing_recommendation` 永远是 `false`，降落永远不会触发。
> 联调时飞控队要能用一条命令测发，例如：
> ```bash
> rostopic pub -1 /mission/state std_msgs/String "data: 'LAND'"
> ```

---

## 2. 位置误差 `target_error`（核心接口）

**话题**：`/vision/target_error`，类型 `geometry_msgs/PointStamped`

| 字段 | 含义 | 单位/范围 |
|---|---|---|
| `point.x` | 水平方向归一化误差 `ex` | −1 ~ +1 |
| `point.y` | 垂直方向归一化误差 `ey` | −1 ~ +1 |
| `point.z` | 检测置信度 `confidence` | 0 ~ 1 |
| `header.stamp` | 图像时间戳 | — |

**计算公式**（代码里就是这么算的）：

```
ex = (目标像素x − 画面宽/2) / (画面宽/2)
ey = (目标像素y − 画面高/2) / (画面高/2)
```

**符号约定**（按图像坐标系，x 向右、y 向下）：

```
        画面左上角 (0,0)
         ┌─────────────────┐
         │                 │
         │      ex<0  ●    │  ← 目标在中心左边：ex 为负
         │        (中心)    │
         │         ● ex>0  │  ← 目标在中心右边：ex 为正
         │   ey<0 ↑         │  ← 目标在中心上方：ey 为负
         │   ey>0 ↓         │  ← 目标在中心下方：ey 为正
         └─────────────────┘
  目标正好在画面中心 → ex≈0, ey≈0
```

**⚠️ 必须和飞控确认的一点**：误差的"方向语义"。
- 当前定义是 **"目标相对无人机在哪"**：`ex>0` 表示目标在画面右侧。
- 若飞控想让无人机**朝目标飞**，控制量应取 `+k·ex`（右偏就往右飞）。
- 若飞控那边约定相反（"无人机相对目标"），取 `−k·ex` 即可，**只需改一个符号，不用改视觉**。
- 请飞控队明确回复用哪种，写进下面第 7 节的确认表。

**是否发布**：只有 `valid=True`（成功检测到图案）时才发布 `target_error`；检测不到时不发。

---

## 3. 任务状态 `mission_state`（飞控 → 视觉）

**话题**：`/mission/state`，类型 `std_msgs/String`，`data` 只认以下 5 个大写成字符串：

| 状态 | 含义 | 视觉的行为 |
|---|---|---|
| `IDLE` | 待机 | 只检测、不算降落 |
| `SEARCH` | 搜索目标 | 检测并输出误差 |
| `FOLLOW` | 跟随/对准目标 | 检测并输出误差 |
| `DROP` | 抛投阶段 | 检测并输出误差（抛投点即平台中心） |
| `LAND` | 降落阶段 | 检测 + 判定 `landing_recommendation` |

> 其它任意字符串会被视觉强制当作 `IDLE`（见 `vision_node.py` 的 `state_cb`）。
> 视觉**不会自己切换状态**，全部由飞控/地面站驱动。

---

## 4. 降落建议 `landing_recommendation`

**话题**：`/vision/landing_recommendation`，类型 `std_msgs/Bool`

只有**同时**满足下面 4 个条件才为 `true`：

| 条件 | 参数 | 当前值 |
|---|---|---|
| 处于降落阶段 | `mission_state == LAND` | 飞控发 `LAND` |
| 连续稳定检测 | `stable_frames` | 8 帧 |
| 误差足够小 | `landing_max_error_normalized` | \|ex\|≤0.08 且 \|ey\|≤0.08 |
| 距离足够近（外圆半径够大） | `landing_min_outer_radius_px` | ≥130 px |

> 第 4 条是**距离阈值**，目前是按经验/粗略标定给的（约对应 1 米）。
> 装机后必须用真实相机重新标定：量几个已知距离，记录对应的外圆半径像素，更新 `vision.yaml`，否则降落高度会不准。

---

## 5. 完整信息 `target_info`（调试 / 地面站用）

**话题**：`/vision/target_info`，类型 `std_msgs/String`，内容是一段 JSON：

```json
{
  "stamp": 1730000000.123,
  "mission_state": "FOLLOW",
  "valid": true,
  "center_px": [321.0, 240.0],
  "outer_radius_px": 141.5,
  "error_normalized": [0.003, -0.002],
  "confidence": 0.92,
  "stable_frames": 15,
  "landing_recommendation": false
}
```

`valid=false` 时，`center_px`、`outer_radius_px`、`error_normalized`、`confidence` 都是 `null`。

---

## 6. 联调前的最小验证（双方各跑一条）

**视觉侧**（香橙派）：
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch uav_vision test_realcamera.launch
```

**飞控侧**（同 ROS master 下）验证能收到误差、能发状态：
```bash
rostopic echo /vision/target_error          # 应看到持续数据
rostopic pub -1 /mission/state std_msgs/String "data: 'LAND'"   # 发降落状态
rostopic echo /vision/landing_recommendation # 对准后应看到 true
```

> ⚠️ 前提是双方在同一个 ROS master（`ROS_MASTER_URI`）下。目前香橙派和电脑网络不通，
> 联调前要先解决组网（同一个路由器/手机热点 + 互相能 ping 通），或用串口桥接。

---

## 7. 双方确认表（签字栏）

| 待确认项 | 视觉侧默认值 | 飞控侧确认 | 结论 |
|---|---|---|---|
| `error` 方向语义 | 目标相对无人机，`ex>0` = 目标在右 | ＿＿＿＿ | ＿＿＿＿ |
| 误差单位 | 归一化 −1~1 | ＿＿＿＿ | ＿＿＿＿ |
| 状态由谁发 | 飞控/地面站发 `/mission/state` | ＿＿＿＿ | ＿＿＿＿ |
| 控制频率 | 跟随图像帧率（约 30Hz） | ＿＿＿＿ | ＿＿＿＿ |
| 降落半径阈值 | 130 px（待装机标定） | ＿＿＿＿ | ＿＿＿＿ |
| ROS 组网方式 | 待定（同网段 / 串口桥） | ＿＿＿＿ | ＿＿＿＿ |

---

## 附：视觉侧一句话接口总结（可直接发给飞控）

> 我订阅你的 `/mission/state`（String：IDLE/SEARCH/FOLLOW/DROP/LAND）；
> 我发布 `/vision/target_error`（PointStamped：x=ex、y=ey、z=置信度，归一化 −1~1，ex>0 表示目标在画面右侧）；
> 还有 `/vision/landing_recommendation`（Bool）告诉你能否降落。
> 检测到目标才发误差，检测不到不发。请确认方向语义和状态由谁发。
