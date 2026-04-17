# teleop_wsad

一个轻量的 ROS 功能包，通过终端键盘的 WSAD 键控制小车的线速度和角速度，发布到 `/cmd_vel`。

使用说明（在已初始化 ROS 环境并启动相应驱动或仿真后运行）:

1. 编译工作区（在工作区根目录）：

```bash
catkin_make
source devel/setup.bash
```

2. 直接运行节点：

```bash
rosrun teleop_wsad wsad_teleop.py
```

或使用 launch：

```bash
roslaunch teleop_wsad wsad_teleop.launch
```

按键：
- W: 前进
- S: 后退
- A: 左转
- D: 右转
- 空格: 停止
- Q: 退出

注意：确保终端具有焦点并且 ROS 网络（MASTER）已正确配置。
