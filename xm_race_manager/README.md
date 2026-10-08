# xm_race_manager —— 厦大省赛任务层（六幕状态机·stage3）

Nav2 之上的大脑。**一个版本两种跑法**，`use_fake_nav` 开关切换：

- **true（VM 演剧）**：不接 Nav2，每航点假开 5 秒，55 秒演完六幕全剧
- **false（实车）**：真调 Nav2 的 NavigateToPose，Nav2 说到才算到

## stage3 改了什么（相比 stage2）

去程从"一发直达射击区"改成 **W1→W6 六航点逐点推进**。
原因：赛道4 上通道是梳齿迷宫（收尾墙/竖井/齿A/B/C/中央结构），
两点直达虽大概率也对，但逐点钉路线 = 每段可控、卡点可见。
回程不变：单发启动区，Nav2 自己规划原路。

## 六幕

| 幕 | 状态 | 干什么 | 进下一幕 |
|---|---|---|---|
| 1 | IDLE | 待命 | 收到 /race/start |
| 2 | GOTO_SHOOT | 按 W1→W6 逐点走（假=每点5s / 真=Nav2逐点） | W6 到达 |
| 3 | SHOOT | 射击（桩B：接口待电控，假3s） | 指令发完 |
| 4 | WAIT_DONE | 等打完（桩C：20s计时兜底） | 超时 |
| 5 | RETURN | 返⚡航（单发启动区 / 真=Nav2） | 到达 |
| 6 | STOP | 停稳保持（终态，规则141行） | 不出去 |

去程航点拓扑（坐标=占位，图纸参考值见代码注释）：
W1 出袋东行 → W2 竖井西侧北上 → W3 绕齿C南下 → W4 齿B东侧回通道 →
W5 齿A西侧南下 → W6 射击区中心。
依据：10-08 标注版图纸蓝线，详见 notes 里 `赛道4_实际路线图.png` v2。

## 目录（放进 VM 的位置）

```
~/smbu_ws/smbu_race_src/xm_race_manager/
├── package.xml
├── CMakeLists.txt
├── README.md
├── launch/xm_race_demo.launch.py
└── src/race_manager_node.cpp
```

## 跑法一：今晚演全剧（不需要 Nav2/雷达/车）

```
cd ~/smbu_ws/smbu_race_src
colcon build --packages-select xm_race_manager     # 编译（第一次几分钟）
source install/setup.bash                          # 登记包
ros2 launch xm_race_manager xm_race_demo.launch.py   # 开关默认true，起
```

新终端发开始信号：

```
source ~/smbu_ws/smbu_race_src/install/setup.bash
ros2 topic pub --once /race/start std_msgs/msg/Empty
```

预期时间线（55 秒全剧）：

```
0s   IDLE -> GOTO_SHOOT（收到开始信号，发 W1）
5s   W1 到 -> 发 W2
10s  W2 到 -> 发 W3
15s  W3 到 -> 发 W4
20s  W4 到 -> 发 W5
25s  W5 到 -> 发 W6
30s  W6 到 -> SHOOT -> WAIT_DONE（[桩B][桩C]提示）
50s  WAIT_DONE -> RETURN（兜底超时）
55s  RETURN -> STOP（假到达）+ 停稳保持
之后不换幕
```

日志里"去程航点 [n/6] xxx"逐条打，哪点没到一目了然。

看完 Ctrl+C。上云：

```
git add -A && git commit -m "任务层stage3: 去程改六航点序列推进" && git push
```

## 跑法二：实车真导航

前提：雷达驱动 + 定位/Nav2 已起（rm_navigation_reality_launch）。然后：

```
ros2 launch xm_race_manager xm_race_demo.launch.py use_fake_nav:=false
```

（`use_fake_nav:=false` = 传参数，冒号等号是 launch 的传参语法）
Nav2 没起时它会打 ERROR 提示先起谁，不崩不装。

## 占位值户口（标完点改哪）

`src/race_manager_node.cpp` 顶部的 `go_wp_` 表：6 行各改 x/y。
回程 `home_x_`/`home_y_` = 启动区出发点。
图纸参考值写在每行行尾注释，只对位置含义，不能直接填
（建图坐标系 ≠ 图纸坐标系）。改完 colcon build 重编译生效。

## 桩（TODO 汇总）

- 桩A：官方暂停/恢复/结束信号——物理形式待公告，动作=叫停一切
- 桩B：射击指令接口——待电控（打靶链问题）
- 桩C：打完判定——无信号源（车不装裁判系统），20s 计时兜底

## 三件套

- 动了什么：新增 xm_race_manager 一个目录，git 可查
- 动不到什么：不 import/不修改学长任何包
- 怎么还原：`rm -rf ~/smbu_ws/smbu_race_src/xm_race_manager` + 重跑 colcon build
