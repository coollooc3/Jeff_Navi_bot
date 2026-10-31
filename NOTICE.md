# 来源与提取说明

本仓库于2026-10-31从`coollooc3/Jeff_follow_bot`的本地工作树提取导航相关包。
源基线提交为`2a3d2aa0f0019415612b92e123f0cd8ece4f799f`；提取包含尚未提交的导航与数据工具进展，
因此不能只用该提交代替本次快照。逐文件来源与导入SHA256分别见`docs/import-manifest.json`中的
`source_sha256`和`sha256`。导入时统一文本LF换行并清理Python尾随空白，Python AST核对不变。
原作者与贡献者信息保留；本仓库是独立初始快照，没有把旧工程全部提交历史改记为新开发。

保留四个原包名：leap_nav_demo、leap_mapping、leap_patrol、leap_localization_lab。
原工程中的视觉跟随、LEAP设备接入、PID诊断及原始实验bag不属于本次导入范围。
根README和Git忽略规则为独立仓库入口整理；包内历史说明保留原开发环境路径与实际验证边界。

主要上游：

- [Jeff Follow Bot](https://github.com/coollooc3/Jeff_follow_bot)
- [Nav2 Humble](https://github.com/ros-navigation/navigation2/tree/humble)
- [TurtleBot3仿真](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble)
- [Gazebo ROS](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2)
- [SLAM Toolbox Humble](https://github.com/SteveMacenski/slam_toolbox/tree/humble)
- [rosbag2 Humble](https://github.com/ros2/rosbag2/tree/humble)

各组件保留原有许可声明；原工作空间根许可证留存于LICENSES，未在提取时统一改写为另一种许可。

2026-11-04 隐私整理：本人邮箱替换为 GitHub noreply 地址；源基线提交号同步到清理后的历史。
导入清单中的 sha256/bytes 对应当前文件，source_sha256 保留整理前来源快照哈希。
