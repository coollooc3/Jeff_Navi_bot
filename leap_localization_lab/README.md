# leap_localization_lab：先看轨迹，再学定位误差

在独立实验入口中复用TurtleBot3、Gazebo、SLAM Toolbox及已有mapping视图。
旧的仿真、导航、巡航和实车跟随入口保持原样。只用于ROS域178、本机通信。

## 离线bag检查工具：当前入口

`exercises/bag_check.py`可直接读取MCAP或SQLite3，统计话题、类型、消息数、首末记录时间、
最大记录间隔，检查指定话题是否有数据；`--patrol`另外解析最后一条巡航状态并检查完成点数。
运行源文件无需build，不创建ROS节点，也不启动仿真或控制机器人。

```bash
# 小例子不依赖ROS环境。
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py --check
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py --check-required
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py --check-patrol

# 读取已有记录；报告应写到输入bag目录外。
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py \
  ~/leap_nav_data/recordings/patrol_normal_mcap_01 \
  --patrol --require /clock /map /scan /odom /tf /tf_static /cmd_vel /patrol/status /amcl_pose \
  --output ~/leap_nav_data/reports/patrol_normal_checked_01.json
```

SQLite3目录使用`--storage-id sqlite3`。上例换成`patrol_cancel_mcap_02`可检查已有取消记录。

| 检查输出 | 含义 |
| --- | --- |
| `required_topic_check` | 要求的话题是否存在且消息数大于0 |
| `patrol.last_status` | 最后一条已录巡航状态，包含state、点数、原因、未决目标及终态标志 |
| `patrol.completion_count_check.passed` | 仅当state为COMPLETED时，完成数是否等于总数 |
| `completion_count_check = null` | 没有巡航状态，未检查；不是检查通过 |
| `patrol.expectation_check` | 指定`--expect-state`后，预期状态、实际状态、通过与否和原因 |

完成点数规则为`state == 'COMPLETED' and completed != total`时报告`completed_count_mismatch`。
正常COMPLETED/3/3与人工CANCELED/0/3都可通过这条规则；检查通过不等同任务成功或已停稳。
它只检查最后一条状态，不扫描所有历史状态矛盾。
退出码0表示所请求的数据检查通过，1表示读取/检查失败，2用于参数或练习检查错误。
点数矛盾时仍保存原状态与检查结果，不改写任务结果为FAILED。

### 按预期任务结果校验

`--expect-state COMPLETED/CANCELED/FAILED`自动启用巡航正文读取，无需另加`--patrol`。
`user_desired_state`来自命令行，实际结果来自最后一条记录的`state`；不选取较早的终态。

```bash
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py \
  ~/leap_nav_data/recordings/patrol_normal_mcap_01 --expect-state COMPLETED \
  --output ~/leap_nav_data/reports/patrol_normal_expected_01.json
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py \
  ~/leap_nav_data/recordings/patrol_cancel_mcap_02 --expect-state CANCELED \
  --output ~/leap_nav_data/reports/patrol_cancel_expected_02.json
echo $?  # 紧接上一条检查命令，查看其退出码。
```

通过要求：最后状态存在、为终态、与预期相同，且明确记录`goal_unresolved=false`；
COMPLETED还要求点数为非负整数且完成数等于总数。取消或失败允许未完成全部点。
缺失状态、仍在运行、未决目标和结果不匹配均退出1，并在指定的JSON报告中保留原状态与失败原因。
如把取消记录声明为预期COMPLETED，将得到`state_mismatch`；这属于受控的不匹配检查，
不是新发生的导航故障。预期FAILED且记录符合时可以退出0，表示测试预期满足，不表示导航成功。
参数非法退出2；未指定`--expect-state`时保留原有数据检查行为。工具不验证实际停车、
源数据新鲜度，也不能证明未录下来的后续行为。

新增纯Python逻辑与CLI检查：

```bash
cd ~/xuegeros_ws/src/leap_localization_lab
python3 -B -m unittest discover -s test -p test_bag_check.py -v
```

2026-10-31验证：新增12项逻辑/CLI测试在Windows Python3.13与Ubuntu Python3.10通过，
原有三组小例子通过。Ubuntu复用已有正常/取消MCAP及无巡航话题的定位记录，完成7组命令检查：
两项预期相符退出0，两项受控结果不匹配退出1，状态缺失退出1，原`--patrol`两项行为保持。
记录文件前后SHA256相同；未启动机器人、仿真或新一轮巡航。内存中反转状态比较条件时，
新测试能够检出错误。需求由双方讨论确定，用户确认预期与实际状态含义，助手实现与验证；
不记为用户独立编写全部工具。证据见[离线验证记录](../docs/evidence/patrol-expectation-20261031.json)。

2026-10-30验证：两份既有巡航MCAP均无误报；纯Python例子覆盖正常完成、点数不足/超出、
取消、失败和运行中；临时合成MCAP的COMPLETED/2/3产生检查失败、JSON和退出码1。
合成反例不是实际巡航故障证据。记录间隔不是丢包率，终态不是停车证明，工具不验证源数据新鲜度。
用户在提示下完成三个读取/统计核心函数，完成点数规则经讨论纠正后由助手接入读取框架与报告。
以下保留定位实验和逐步学习过程；当前工具用法以上方入口为准。

## 这次改变什么

原burger模型没有设置`odometry_source`。Gazebo ROS 3.9.0插件默认从世界位姿生成odom；
本入口复制安装模型，在副本中显式选择`0`，由仿真轮关节速度积分得到odom。
它仍是模拟轮速，不是真实编码器数据；理想仿真中不保证出现明显漂移。

世界状态插件仅加入本次world副本。`world_pose`以10Hz请求
`/lab/gazebo/get_entity_state`，把burger的世界位姿发布为`/lab/ground_truth`。
使用服务答复中的仿真时间戳，不使用回调抵达时刻，不广播额外TF，不发布速度。
SLAM继续消费轮速odom与激光。世界位姿只用于记录，不提供给SLAM进行定位。

接口依据：Gazebo ROS 3.9.0官方
[差速插件](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.9.0/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)
和[世界状态插件](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.9.0/gazebo_ros/src/gazebo_ros_state.cpp)。

## 构建

```bash
cd ~/xuegeros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select leap_localization_lab --symlink-install
source install/setup.bash
```

## 终端A：仿真、SLAM与记录

先退出旧Gazebo、建图、Nav2、巡航及键盘会话。小车、热点和Agent不参与。

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 launch leap_localization_lab odometry_lab.launch.py
```

启动Gazebo、RViz、SLAM、只读世界位姿节点和rosbag记录。
看到`WORLD REFERENCE READY`，且Gazebo/RViz中的机器人及雷达正常后再驾驶。
只关GUI可用`gui:=false rviz:=false`；不影响数据记录。

## 终端B：自己驾驶

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.15 -p turn:=0.5
```

`i`前进，`j/l`转向，`u/o`前进转向，`k`停车；松开按键不会自动停车。
本轮是TurtleBot3仿真，实车的转动限制不直接套用到它。
沿空闲通道绕一小段，回到起点附近即可，不需要精确闭合，也不必填满整张地图。
切换窗口前按`k`。本轮不发Nav2目标，不同时启动其他速度发布者。

完成后先按`k`、退出B，再在A按Ctrl+C并等待rosbag退出。
本轮不要重置Gazebo时间或重新摆放模型；重来时退出后新开一轮。

## 数据与后续分析

A会打印`LAB DATA: /home/chuyun/leap_nav_data/localization/<本轮编号>`。
目录保存原始/副本模型哈希、来源说明、模型及world副本，以及`bag/`：

| 数据 | 含义 |
| --- | --- |
| `/odom` | 轮关节速度积分的位姿 |
| `/lab/ground_truth` | Gazebo世界位姿，带仿真时间 |
| `/scan` | 原始激光 |
| `/tf`、`/tf_static` | 包含SLAM校正的坐标关系 |
| `/clock` | 仿真时钟 |

```bash
ros2 bag info ~/leap_nav_data/localization/<本轮编号>/bag
```

先检查话题数量、时间覆盖和时间戳，再对齐到同一参考系比较轨迹；
原始bag不预先对齐、不逐帧消除误差。本包目前不输出ATE/RPE，
也不把保存bag或静止启动检查当成行驶精度验证。离线分析与二维配准练习随后接入。

## 逻辑检查

```bash
cd ~/xuegeros_ws/src/leap_localization_lab
python3 -m unittest discover -s test -v
```

检查实验模型确实改为encoder、状态插件来源、安装原件保持完整及会话不覆盖。
实际构建、静止启动与人工行驶结果分别记录，不混为同一种证据。

2026-10-28准备检查：Ubuntu构建及6项逻辑检查通过；Windows检查5项通过，
依赖lxml的解析检查留给Ubuntu执行。静止启动收到odom、世界位姿、scan和map，
记录中6类话题均非空，包含map→odom及odom→base_footprint，bag使用仿真时间。
该会话没有发布运动命令，结束后确认Gazebo端口释放。首次检查暴露的XML声明
与spawn_entity解析器不兼容问题已修正并加入回归检查；失败记录保留。
此处仅证明启动及数据链路，人工行驶轨迹与精度分析尚未进行。

## 2026-10-29：从已有数据开始个人代码练习

用户随后已完成会话`20261029T070856Z-72403e`；离线轨迹对比见仓库外的
`output/localization-run-20261029-72403e/`。理想仿真中odom已很准，不能据此宣称SLAM更优。

首个练习是`exercises/point_transform.py`中的`transform_points`：只填
`x_target`和`y_target`两行，完成从后一次激光坐标系到前一次激光坐标系的二维变换。
输入数据`exercises/scan_pair.json`来自同一bag的115.987s和116.987s，保留bag哈希、
帧名和参数来源；前后分别333/324个有效点。相对位姿来自扫描时刻插值的odom，
并计入静态雷达安装偏移；它是初值，不是真值。此步没有求匹配、ICP或SLAM校正。

不需要启动Gazebo、ROS节点、小车或Agent，也不需要重新build：

```bash
code ~/xuegeros_ws/src/leap_localization_lab/exercises/point_transform.py
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/point_transform.py --preview
# 填完两行后运行：
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/point_transform.py
```

`--preview`生成原始两帧叠画的`scan_preview.svg`；完成后先检查4个小例子，
再生成变换前后的`scan_alignment.svg`。SVG可用浏览器或VS Code预览。
两帧可见表面不同，无须每个点完全重合；相同射线编号不代表同一物理点。
本轮仅检查了练习框架及临时参考实现，正式文件仍保留TODO，用户实现与运行待完成。

随后用户完成`transform_points`两行并运行通过4个小例子；回读Ubuntu源码，确认变动
仅为预期的两个表达式，已同步到Windows仓库工作树。此结果证明当前小例子及接口通过，
不扩大为配准或完整SLAM算法已经掌握。

第二个练习是`exercises/nearest_neighbor.py`中的`nearest_point`循环体：
在同一坐标系中遍历参考点，返回距离最小的点及平方距离。空参考返回`None, inf`；
距离相同时保留第一个候选。最近点只是候选对应，不保证同一物理点，允许多个源点选中同一参考点。

```bash
code ~/xuegeros_ws/src/leap_localization_lab/exercises/nearest_neighbor.py
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/nearest_neighbor.py
```

填完先运行4个小例子，再复用第一题的变换函数，在真实仿真扫描上比较odom初值与人为
增加X方向10cm、朝向5度偏差的初值。打印的是候选点最近距离的RMS，不是位姿误差，
也没有运行离群点剔除或位姿优化。这是离线受控初值变化，不是新实车或仿真驾驶实验。
程序当前保留第二题TODO；遍历搜索用于理解流程，不作为高性能最近邻实现。

随后用户完成最近点循环，4个小例子通过；输出原始odom初值的候选RMS为9.083cm，
人为增加10cm X与5度角度偏差后为14.154cm。回读源码确认只完成预期循环，并同步Windows工作树。
原始初值下候选距离中位数约1.36cm，12/324个候选超过20cm；RMS受少数较大距离影响，
不将其解释为机器人定位误差，也不把候选最近点当作已确认的同一物理点。

第三题`exercises/translation_alignment.py`先固定角度，只在人为X+10cm的初值下修正平移。
用户实现`estimate_translation(pairs)`：对同一坐标系中的固定对应点求`target-source`
平均值，返回`dx, dy`。框架重复“变换原始点→重新寻找最近点→保留距离不超过20cm的候选
→估计修正→更新X/Y”，最多20轮；这个阈值是离线对应点筛选参数，与实车安全距离无关。

```bash
code ~/xuegeros_ws/src/leap_localization_lab/exercises/translation_alignment.py
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/translation_alignment.py
```

小例子验证固定对应关系下的平移计算，真实扫描只是近邻候选；角度暂不优化。
`translation_result.json`保存迭代记录。与原始odom初值的差值仅说明已知人工偏差的消除情况，
不能称为真实位姿误差或完整ICP/SLAM验收。第三题核心实现仍留给用户。

## MCAP第一步：自己完成已有bag的格式转换

按2026-10-30岗位准备安排，近期优先已有Nav2数据的录放与检查；算法练习保留。
本次只读确认源bag仍在，共26,128条消息、6个话题、170.5秒，存储为sqlite3。
Ubuntu已提供`ros2 bag convert`，MCAP插件尚未安装；软件源已有候选版本。
以下为首次操作流程；完成状态见本节末尾。安装插件时Ubuntu需要联网，可通过热点提供网络；
离线转换无需外网，也无需小车、Gazebo或Agent。

```bash
sudo apt update
sudo apt install ros-humble-rosbag2-storage-mcap

source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
python3 -c "import rosbag2_py; print(rosbag2_py.get_registered_writers())"
```

列表中应出现`mcap`。安装有错误时先处理错误，保留输出。
查看已提供的练习配置：

```bash
code ~/xuegeros_ws/src/leap_localization_lab/exercises/convert_to_mcap.yaml
```

`uri`是新的输出目录，`storage_id`选择MCAP插件；`all: true`表示复制输入bag的全部话题，
不表示开始订阅和录制全部实时话题。YAML中的输出目录应尚不存在；重复练习时先在VS Code
换一个新输出目录，并在后续info命令中使用同一路径，保留原结果。

```bash
ros2 bag convert \
  -i ~/leap_nav_data/localization/20261029T070856Z-72403e/bag \
  -o ~/xuegeros_ws/src/leap_localization_lab/exercises/convert_to_mcap.yaml

ros2 bag info ~/leap_nav_data/localization/20261029T070856Z-72403e/bag_mcap
```

`-i`指定已有输入bag，`-o`指定转换配置YAML；此处不是输出文件路径。
先核对Storage id为mcap、总数26,128，各话题数量依次为odom 5,008、scan 851、tf 16,860、
tf_static 1、ground_truth 1,703、clock 1,705。文件大小不要求与SQLite一致。
原始bag使用仿真时钟，info显示1970年附近是接近零的仿真时间被按日期显示，并非采集发生在1970年。
此步仅转换并查看摘要，隔离回放与自写检查工具随后推进。
参考：[MCAP官方ROS2指南](https://mcap.dev/guides/getting-started/ros-2)。

2026-10-30用户已执行转换并提供`ros2 bag info`：Storage id为mcap，输出`bag_mcap_0.mcap`，
9.1MiB，170.5秒，26,128条消息；6个话题的类型与数量均与原SQLite摘要一致。
此项证明转换后的存储可读取且摘要匹配；回放与自写检查工具尚待进行。
当前bag没有`/map`、`/cmd_vel`和`/patrol/status`，后续巡航任务记录按分析目标补充。
记录的是选定话题消息，回放是按时间重新发布这些消息，不会自动重新运行原节点、Gazebo或完整导航任务。

## MCAP第二步：回放激光与TF

2026-10-30准备核查未发现运行中的Gazebo、SLAM、RViz或播放器；MCAP中包含录制的
`/clock`，并保留`/tf_static`的可靠/Transient Local QoS。本节为人工回放练习，尚未记为已观察通过。
`exercises/bag_view.rviz`只显示网格、激光和TF，固定坐标系为odom。

终端A先打开RViz：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 run rviz2 rviz2 \
  -d ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_view.rviz \
  --ros-args -p use_sim_time:=true
```

终端B再打开播放器：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 bag play ~/leap_nav_data/localization/20261029T070856Z-72403e/bag_mcap \
  --rate 2.0 --start-paused
```

在B终端按空格开始/暂停/继续；切回RViz观察。`--rate 2.0`为两倍速，`--start-paused`
使启动后等待用户按键。播放器读取文件并发布消息，RViz订阅显示，实时仿真无需启动。
本轮直接回放原`/clock`，不额外加`--clock`生成第二个时钟；RViz使用`use_sim_time=true`。
在播放器开始前提示odom不存在是尚未收到TF，开始后再判断是否持续异常。

原记录运动开始约在仿真94.5秒，两倍速下前约47秒基本静止；不会为了跳过这段直接使用
start-offset，因为文件早期仅有一条静态TF。观察激光点和TF坐标随记录更新，以及暂停/继续
是否生效；本bag没有地图或机器人描述，视图也不添加这些显示。Ctrl+C结束播放器，再关闭RViz。

用户随后反馈首次回放及暂停/继续均无问题，记为本次人工可视化检查通过；不等同重新执行完整导航。

## MCAP第三步：本人编写流式统计核心

`exercises/bag_check.py`使用Humble的`rosbag2_py.SequentialReader`只读读取文件，
不创建ROS节点、不调用spin、不回放或发布消息；无需设置通信域，读取MCAP需要已source ROS环境。
读取框架每次得到`topic_name, serialized_data, record_time_ns`，按话题保留一个统计字典。
本轮不解析消息正文，因此时间指的是bag的记录时间，不是`header.stamp`。

用户完成`update_topic_stats(stats, record_time_ns)`：直接更新传入字典，无需返回新字典。
字典字段为count、first_record_ns、last_record_ns、max_gap_ns；空话题时间为None，
单条消息还没有间隔，max_gap_ns仍为None；相同时间戳允许，间隔为零。
先用上一条时间计算间隔，再更新last；倒退时间戳由现有检查拒绝。所有内部时间为整数纳秒。

```bash
code ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py --check
```

完成小例子后，直接检查真实文件并保存JSON到bag目录外：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py \
  ~/leap_nav_data/localization/20261029T070856Z-72403e/bag_mcap \
  --output ~/leap_nav_data/reports/localization_mcap_stats.json
```

这份bag的总数应为26,128，各话题与info一致；`--storage-id sqlite3`可用于原SQLite bag。
最大记录间隔不自动证明丢包；本bag用仿真clock录制，记录时间也可能相同，不能把相同时间戳当作重复消息。
框架提供五个小例子，用户实现仍留空；验收需区分临时参考实现验证与用户运行结果。

准备验证：临时参考实现通过5个小例子，并在Ubuntu以普通用户读取真实MCAP，
六话题数量/类型与metadata一致、总数26,128；学生文件保持TODO。框架验证记录见
仓库外`output/bag-check-framework-20261030/`。此项不是用户本人完成统计函数的证据。

随后用户在示例骨架提示下完成统计函数，5个小例子及真实MCAP运行均通过。
已回读用户源码与JSON：只改变预期的函数实现，JSON与参考统计一致；用户版本同步到Windows工作树。
初版源码快照与用户报告保存在仓库外同一验证目录。此结果不扩大为所有bag或源时间质量已验证。

接着增加一个可选练习：`find_missing_topics(topic_stats, required_names)`。
输入是报告里的话题字典，以及本次分析所需的话题名列表；返回不存在或消息数为0的名称，保持要求顺序。
当前函数仍留TODO；不使用`--require`时，既有统计命令仍可正常使用。

```bash
python3 bag_check.py --check-required
# 填好新函数后，用当前bag做一次预期缺项检查：
python3 bag_check.py \
  ~/leap_nav_data/localization/20261029T070856Z-72403e/bag_mcap \
  --require /scan /odom /map \
  --output ~/leap_nav_data/reports/localization_required_topics.json
```

当前文件未录map，故应提示`missing or empty: /map`，保存含检查结果的报告并以退出码1结束；
这表示未满足指定分析需求，不表示程序崩溃或原机器人没有发布地图。要求全部满足时退出码0。

随后用户完成必需话题检查函数，3项小例子通过；真实bag准确报告缺少`/map`并保存JSON。
回读源码确认只有新函数改变，报告与基础统计及指定要求一致；用户版本已同步Windows，
初版和扩展版快照均保存在仓库外验证目录。用户条件表达式中的额外成员检查可精简，但逻辑正确，本轮保留。

## MCAP第四步：录制一轮真实运行的仿真巡航

这里“真实运行”指实际启动Gazebo/Nav2并录制本轮消息，仍然是仿真证据。
2026-10-30只读核对Ubuntu安装入口可用、默认路线为A→B→C；发现旧bag播放器和RViz尚在运行，
应先由用户退出。新目录`~/leap_nav_data/recordings/patrol_normal_mcap_01`当时尚不存在。
此流程为用户待执行指令，不记为已录制成功。

每个新终端先source Humble和工作区，设置`ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1`。
本轮保留已有入口的域号约定，域号范围统一调整作为单独事项。

1. A：`ros2 run leap_nav_demo run_manual --gui`，等待`READY:`。
2. B：`ros2 launch leap_patrol patrol.launch.py`，初始IDLE。
3. C：先开始录制，再下达巡航任务：

```bash
mkdir -p ~/leap_nav_data/recordings
ros2 bag record -s mcap --use-sim-time \
  -o ~/leap_nav_data/recordings/patrol_normal_mcap_01 \
  /clock /map /scan /odom /tf /tf_static /cmd_vel /patrol/status /amcl_pose
```

4. D：`ros2 service call /patrol/start std_srvs/srv/Trigger '{}'`，然后
`ros2 topic echo /patrol/status --full-length`观察至COMPLETED。
服务success仅表示请求被接受，最终结果看状态。若FAILED，保留记录与原因，不把失败数据直接丢弃。
5. D退出echo，C按Ctrl+C停止录制并等待退出；A/B保留供同会话取消任务练习。

```bash
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py \
  ~/leap_nav_data/recordings/patrol_normal_mcap_01 \
  --require /clock /map /scan /odom /tf /tf_static /cmd_vel /patrol/status /amcl_pose \
  --output ~/leap_nav_data/reports/patrol_normal_mcap_01.json
```

本轮强调如何选择记录内容及把任务状态与数据对应；直接录制MCAP，不再从SQLite转换。
这份新记录预期包含旧几何实验未录的地图、命令、应用状态与定位估计；是否完整由实际输出判断。

用户已完成正常巡航录制并运行检查，九项必需话题通过，总数9,854。
已只读解析bag：记录时间36.5s开始A，49.2s转B，77.4s转C，98.8s为COMPLETED、完成3/3、
最后Nav2结果SUCCEEDED、goal_unresolved=false；末条cmd_vel为零，记录结束前2秒的odom速度接近零。
以上为记录中的任务结果与速度估计，不增加精确停车或无碰撞结论。
AMCL最大记录间隔为26.2..37.4s，其中到36.5s前任务尚未启动，36.6s才记录到明显命令/odom运动；
不能将该11.2秒直接作为定位断流证据。只读实时参数查询未发现节点，未据此判定原运行无AMCL，
进程核查仍见Nav2和巡航进程；此项不影响已保存数据的离线解析。
报告、状态时间线及原bag归档见仓库外`output/patrol_normal_*`。下一轮取消复用会话，
新目录使用`~/leap_nav_data/recordings/patrol_cancel_mcap_01`，准备核查时尚不存在。

取消录制首次准备遇到目录重复。只读确认`patrol_cancel_mcap_01`已存在但为0消息、0秒，
当前无残留录制进程；保留该空记录，下一次改用`patrol_cancel_mcap_02`，检查报告也对应改名。
上轮导航`session.json`明确为`session_time_limit`、READY后1800秒、ok=true，是会话预算到期后的正常清理。
用户已重启A/B；等待新READY后开始新录制，再启动与取消巡航。以后需要较长会话可用
`run_manual --gui --seconds 3600`，已有会话无需为改预算再次重启。

用户随后完成`patrol_cancel_mcap_02`：2,900条消息，九项必需话题均有数据。
只读解析记录得到`IDLE → RUNNING → CANCELING → CANCELED`，取消发生在A，
完成0/3点，原因`operator_cancel`，最终Nav2结果CANCELED、goal_unresolved=false、
active_goal_uuid=null；此后未记录到向B/C推进的状态。末条cmd_vel为零，
末尾2秒的60条odom中线速度与角速度估计均接近零。

CANCELING与CANCELED在bag中的记录时间均为539.7s；本轮用仿真clock录制，
不能据同一记录刻度推出取消耗时为零。正常完成与人工取消都出现零速命令，
任务结束类型须结合状态消息判断。必需话题检查只确认存在且有消息，不能代替正文检查。
两份原记录、对比JSON和速度图归档在仓库外`output/patrol_recording_*20261030*`；
取消原bag归档为`output/patrol_cancel_mcap_02_20261030.tar.gz`。
正文解析与画图由助手完成，不记为用户已独立编写的检查功能。
本轮采集结束，后续离线学习消息反序列化、String.data中的JSON解析及任务终态提取，
不再要求重复启动Gazebo或额外驾驶采集。

## MCAP第五步：读取巡航状态正文

仍使用`exercises/bag_check.py`，不重启仿真。新增可选`--patrol`，默认统计功能不变。
读取链为`reader.read_next()`得到序列化字节，用`deserialize_message(..., String)`还原ROS消息，
取`message.data`的JSON字符串，再由练习函数`parse_patrol_status(status_text)`解析。
该函数仅留两处填空：`json.loads`转换为字典，以及判断state是否为COMPLETED/CANCELED/FAILED。
后面的字段提取已提供；goal_unresolved原样保留，终态不自动代表已停稳。

```bash
code ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py --check-patrol
```

填空完成后才使用`--patrol`读取实际文件；纯Python小例子不需要ROS或DDS通信。
真实bag读取需要source Humble；它仍不创建节点、不发布消息、不需要Gazebo。

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
python3 ~/xuegeros_ws/src/leap_localization_lab/exercises/bag_check.py \
  ~/leap_nav_data/recordings/patrol_cancel_mcap_02 --patrol \
  --output ~/leap_nav_data/reports/patrol_cancel_status_02.json
```

解析结果只代表文件中最后一条状态，不回溯挑选较早的终态：若新一轮仍RUNNING，
就报告RUNNING及is_terminal=false，不沿用前一轮COMPLETED。
未录到巡航状态时输出unknown、last_status=null并退出1；正文损坏或不符合所需字段时明确报错。
正常读取的退出码0表示分析执行成功，FAILED任务也可以被成功分析，不能用该退出码替代任务结果。

准备验证：临时参考实现通过八项状态例子和损坏JSON拒绝检查，错误终态判据能被例子检出；
在Ubuntu读取正常/取消/无巡航话题三个现有MCAP，结果与既有报告一致，原统计与必需话题检查不变。
现有两个用户函数AST保持一致；交付文件保留本轮两处TODO，尚无用户完成本轮函数的证据。
验证日志见仓库外`output/patrol_status_framework_20261030.log`。

随后用户填写JSON转换与终态成员判断，运行八项小例子均通过。
已回读Ubuntu源码，核对仅`parse_patrol_status`函数改变并同步Windows工作树；
该函数的终态集合为COMPLETED/FAILED/CANCELED。本轮使用用户版本读取实际MCAP的结果尚待提供，
不将前述助手临时参考实现的文件检查混作用户执行结果。

用户随后用本人完成的函数读取两份实际MCAP并保存JSON：正常记录为COMPLETED、C、3/3，
取消记录为CANCELED、A、0/3、operator_cancel，二者均is_terminal=true、goal_unresolved=false。
已回收两份用户报告并核对总数9,854/2,900、巡航消息99/34及最终字段，与终端输出一致。
报告保存于仓库外`output/bag-check-framework-20261030/student_patrol_*_status_*.json`。
本轮已完成格式转换/直接录制、回放、统计、必需话题检查和终态提取的人工练习闭环；
这些结果不等同停车验证、独立完成整个工具或具备完整SLAM算法开发能力。
