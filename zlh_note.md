# 自主建图框架梳理笔记
* 路径规划
    * Dijkstra
        * 用来寻找图形中节点之间的最短路径
        * 优先级队列以每个节点距离起点的总移动代价排序
        * 每次从优先级队列中选取代价最小的作为下一个遍历的节点
        * 不考虑节点移动代价差异时，退化为BFS
    * 最佳优先搜索
        * 优先级队列以每个节点到终点的距离为优先级
        * 每次选取到终点移动代价最小的节点作为下一个遍历的节点
        * 缺点：如果起点和终点之间存在障碍物，则最佳优先算法找到的很可能不是最短路径
    * A*
        * ![](https://notes.sjtu.edu.cn/uploads/upload_2f85834aeb3cb066dddba4cdf3433461.png)
            * f(n)是节点n的综合优先级。当我们选择下一个要遍历的节点时，我们总会选取综合优先级最高（值最小）的节点。
            * g(n) 是节点n距离起点的代价
            * h(n)是节点n距离终点的预计代价，这也就是A*算法的启发函数。
        * 启发函数：控制算法的速度和精确度
            * 在极端情况下，当启发函数h(n)始终为0，则将由g(n)决定节点的优先级，此时算法就退化成了Dijkstra算法。
            * 如果h(n)始终小于等于节点n到终点的代价，则A*算法保证一定能够找到最短路径。但是当h(n)的值越小，算法将遍历越多的节点，也就导致算法越慢。
            * 如果h(n)完全等于节点n到终点的代价，则A*算法将找到最佳路径，并且速度很快。可惜的是，并非所有场景下都能做到这一点。因为在没有达到终点之前，我们很难确切算出距离终点还有多远。
            * 如果h(n)的值比节点n到终点的代价要大，则A*算法不能保证找到最短路径，不过此时会很快。
            * 在另外一个极端情况下，如果h(n)相较于g(n)大很多，则此时只有h(n)产生效果，这也就变成了最佳优先搜索。
            * 对于网格形式的图，有以下这些启发函数可以使用
                * 如果图形中只允许朝上下左右四个方向移动，则可以使用曼哈顿距离（Manhattan distance）。
                * 如果图形中允许朝八个方向移动，则可以使用对角距离。
                * 如果图形中允许朝任何方向移动，则可以使用欧几里得距离（Euclidean distance）。

    * A*和Dijkstra比较
        * Dijkstra算法计算源点到其他所有点的最短路径长度，A*关注点到点的最短路径(包括具体路径)。
        * Dijkstra算法建立在较为抽象的图论层面，A*算法可以更轻松地用在诸如游戏地图寻路中。
        * Dijkstra算法的实质是广度优先搜索，是一种发散式的搜索，所以空间复杂度和时间复杂度都比较高。对路径上的当前点，A*算法不但记录其到源点的代价，还计算当前点到目标点的期望代价，是一种启发式算法，也可以认为是一种深度优先的算法。
        * 由第一点，当目标点很多时，A*算法会带入大量重复数据和复杂的估价函数，所以如果不要求获得具体路径而只比较路径长度时，Dijkstra算法会成为更好的选择。
* 局部避障
    * dwa（动态窗口法）
        * 原理主要是在速度空间（v,w）中采样多组速度，并模拟出这些速度在一定时间内的运动轨迹，并通过评价函数对这些轨迹进行评价，选取最优轨迹对应的速度驱动机器人运动。
        * 动态窗口法 DWA 的实现包含两个步骤
            * 对机器人速度进行约束限制，形成动态窗口进行速度采样；
            * 根据速度采样点求出待评价轨迹，最大化评价函数选取最优速度命令。
        * 动态窗口是由一系列的约束构成，其中约束主要包括差动机器人的非完整约束、环境障碍物约束与受结构与电机影响的动力学约束。
        * 优点
            * 计算复杂度低：考虑到速度和加速度的限制，只有安全的轨迹会被考虑，且每次采样的时间较短，因此轨迹空间较小
            * 可以实现避障：可以实时避障，但是避障效果一般
            * 适用于差分和全向车模
        * 缺点
            * 前瞻性不足：只模拟并评价了下一步，如在机器人前段遇见“C”字形障碍时，不能很好的避障
            * 动态避障效果差： 模拟运动轨迹断，动态避障效果差
            * 非全局最优路径： 每次都选择下一步的最佳路径，而非全局最优路径不适用于阿克曼模型车模
    * teb（时间弹力带）
        * TEB全称为Time Elastic Band，定义橡皮筋连接起始、目标点，并让这个路径可以变形，变形的条件就是将所有约束当做橡皮筋的外力。关于time eletic band的简述：起始点、目标点状态由用户/全局规划器指定，中间插入N个控制橡皮筋形状的控制点（机器人姿态），当然，为了显示轨迹的运动学信息，我们在点与点之间定义运动时间Time，即为Timed-Elastic-Band算法。通过此方法可以把问题描述为一个多目标优化问题，通过构建超图(hyper-graph)，使用g2o框架中的图优化来求解
        * 优点
            * 适用于各种常见车模：如差分、全向、阿克曼模型
            * 有很强的前瞻性： 对前方一段轨迹进行优化
            * 动态避障： 对动态障碍有较好的避障效果，可直接使用其封装好障碍类Obstacle 如：静态障碍时TEB算法轨迹规划效果 
        * 缺点
            * 计算复杂度较大：可通过牺牲预测距离来降低复杂度
            * 速度和角度波动较大、控制不稳定：源码中是通过两状态之间的距离和角度差及时间差来计算该控制周期内的速度和角速度，使得在控制过程中速度和角度波动较大，如下图所示。在计算资源足够的情况下，提高控制频率可以改善此现象。 
    * dwa和teb比较
        * teb在运动过程中会调整自己的位姿朝向，当到达目标点时，通常机器人的朝向也是目标朝向而不需要旋转。dwa则是先到达目标坐标点，然后原地旋转到目标朝向。
* 建图
    * **gmapping**算法
* 探索
    * ***frontier_exploration***
        * 好像软件包在ros noetic 不能用呢:(
    * ***ergodic_exploration***
        * 相比于其他算法，ergodic_exploration算法具有一些独特的优点和缺点。下面是对ergodic_exploration算法与其他算法进行比较的优缺点：
            **优点：**
            1. 全面覆盖：ergodic_exploration算法旨在使机器人尽可能地覆盖整个环境的状态空间。它通过随机采样和避障技术，让机器人能够探索不同的区域，从而获取更全面的环境信息。

            2. 无需先验知识：ergodic_exploration算法不依赖于环境的先验知识。它使用随机采样进行探索，因此可以应用于各种未知环境，而无需提前了解或建立环境模型。

            3. 简单实现：ergodic_exploration算法相对简单，并且容易实现。它主要由随机采样、路径规划和避障等基本步骤组成，没有过于复杂的算法逻辑。

            **缺点：**
            1. 低效性：由于ergodic_exploration算法的随机性质，它可能导致机器人做出一些不必要的移动或行走回头路的情况。这可能会使算法的效率降低，导致探索时间增加。

            2. 避障困难：ergodic_exploration算法在遇到复杂的环境或障碍物布局时可能面临避障困难。尽管算法中包括了避障技术，但在一些复杂的场景下，机器人可能会遇到无法找到清晰的路径的情况。

            3. 未考虑目标优先级：在ergodic_exploration算法中，所有的状态都被视为同等重要，机器人会随机选择目标状态进行探索。这可能导致机器人在探索过程中不会特别关注某些重要的目标或区域。
        * 在实验室仿真环境下的实现流程（ubuntu20.04 noetic）
            * 配置环境
            * 编译
           ``` catkin build ```
            * 打开仿真场景
            ```roslaunch ranger_mini_gazebo ai_robot_ranger_gazebo.launch```
            * 启动局部避障
            ```roslaunch ai_robot_nav_demo obs_avoid.launch```
            * 启动速度平滑
            ```roslaunch yocs_velocity_smoother standalone.launch```
            * 进行自身定位及全局路径规划与gmapping建图
            ```roslaunch ai_robot_nav_demo brain_explore_independent.launch ```
            * 启动ergodic_exploration探索
           ``` roslaunch ai_robot_nav_demo explore.launch```
            * 保存探索地图
            ```roslaunch ai_robot_nav_demo save_map.launch```
            **锐评：**
            较为初级的自主探索算法，探索路径容易卡墙，但好处是不卡墙的时候探索率很高
    * ***explore_lite***
        * explore_lite 提供了贪婪的基于边界的探索，直到找不到边界，
        * 移动命令会发送至 move_base 节点。explore_lite ****不会创建自己的成本图，这使得配置更容易，效率更高（资源更少）****。Node 只是订阅nav_msgs/OccupancyGrid 消息。
        * 节点可以进行边界过滤。目标黑名单允许处理机器人无法进入的地方，也就是说可以在 move_base 的代价地图层中配置禁止层参数，explore-lite 也会识别到，并不去探索这部分边界。
        * ![](https://notes.sjtu.edu.cn/uploads/upload_e60c149b100f89bcdc867f9ff8381d12.png) explore_lite订阅nav_msgs/OccupancyGrid和map_msgs/OccupancyGridUpdate消息来构建一个地图，它会在其中寻找边界，可以使用 move_base 发布的成本地图（即<move_base>/global_costmap/costmap），也可以使用由SLAM算法构建的地图。然后explore-lite算法发布move-base移动指令给move-base节点，控制小车运动。需要提供的tf变换：global_frame → robots_base_frame。这种变换通常由映射算法提供。这些框架通常称为map和base_link。可以在launch文件中调整robot_base_frame名称。不需要设置global_frame。global_frame的名称将自动来自costmap_topic。
        * 在实验室仿真环境下的实现流程（ubuntu20.04 noetic）
            * 配置环境
            * 编译
           ``` catkin build ```
            * 打开仿真场景
            ```roslaunch ranger_mini_gazebo ai_robot_ranger_gazebo.launch```
            * 启动局部避障
            ```roslaunch ai_robot_nav_demo obs_avoid.launch```
            * 启动速度平滑
            ```roslaunch yocs_velocity_smoother standalone.launch```
            * 进行自身定位及全局路径规划与gmapping建图
            ```roslaunch ai_robot_nav_demo brain_explore_independent.launch ```
            * 启动explore_lite探索与rviz
           ``` roslaunch ai_robot_nav_demo explore.launch```
            * 保存探索地图
            ```roslaunch save_map_explore_lite.launch```
        * 评价：
            * 优点：
                * 可以较快地获取环境的大致地图
                * 探索路径较为合理，不容易卡墙（应该是实验室第二套算法局部避障的优点）
                * 只要给足时间就可以实现全覆盖探索建图
                * 所需计算量较小
            * 缺点：随着时间增加，探索的效率下降，逐渐开始走回头路
            * 自主探索生成地图
                * world6（about 15min）
                    * ![](https://notes.sjtu.edu.cn/uploads/upload_a44e9eed9e8d61099c83e5d86a83aa81.png)
                * world5（5min）
                    * ![](https://notes.sjtu.edu.cn/uploads/upload_4d019253b05ae7612b5beefaf16bb28b.png)

            * rqt_graph
                * ![](https://notes.sjtu.edu.cn/uploads/upload_ca28ce1c20b23dd7af0a488814ad85ea.png)

    * ***rrt_exploration***
        * 基于2D的，通常采用图像算法的边缘检测来检测已知区域与未知区域的边界。基于Rapidly-exploring Random Trees（快速搜索随机树RRT）的探索策略。由于RRT基本上是朝向未知区域的（unexplored and unvisited），并且RRT可以扩展到更高维区域。同时采用local tree与global tree来检查边缘点，使得机器人的exploration更加高效
        * 一旦检测到边缘后，就会取其中心为目标点，然后让机器人去探索该点。而为了检测边缘点，需要对整张地图进行处理，而这个操作通常是耗时的，为此大量的研究人员花费精力在检测frontier edges的效率上。
        * 本包中，RRT树只是用于search边缘点，而检测到的边缘点经过滤波就会依次安排给机器人。当机器人接收到point时，就会运动到对应的点。在此期间，机器人上的传感器将会扫描建图。而通过多个独立的RRT树来加速边缘点的检测，则是本包的创新点。
        * 如下图所示，这个功能包主要分为三个模块。
            1. 基于RRT的边界检测模块（负责检测边界点，分为全局检测节点和局部检测节点以及基于opencv的边界检测器节点）
            2. 滤波模块（存储边界点，并通过mean shift算法来聚类，检测出无效以及旧的边界点）
            3. 以及task allocator模块（接收到滤波模块传来的边界点后，分配到机器人）
        * 本包还需要与SLAM及path planning模块(move_base)相结合来使用。
        * ![](https://notes.sjtu.edu.cn/uploads/upload_7375ecf8393d1dbfd0326523f703b065.png)
        * ![](https://notes.sjtu.edu.cn/uploads/upload_a1f2df8a44bd452a74af843f9e847f93.png)
        * 重点关注信息增益的使用
        * 优点
        * 缺点
            * 使用sklearn等速度不是很理想
        * 在实验室仿真环境下的实现流程（ubuntu20.04 noetic python39）
            * 方案1：直接使用navigation包中的路径规划
                * 配置环境
                * 编译
               ``` catkin build ```
                * 启动gazebo仿真场景
                ```roslaunch ranger_mini_gazebo ai_robot_ranger_gazebo.launch```
                * 启动move_base节点
                ```roslaunch robot_navigation move_base.launch```
                * 启动gmapping建图
                ```roslaunch robot_navigation gmapping.launch```
                * 启动rrt_exploration探索
               ``` roslaunch rrt_exploration simple.launch``
                * 保存探索地图
                ```roslaunch save_map.launch```
                * 可以尝试用dwa取代teb，A*取代Dijkstra
                * 自主探索生成地图(有时卡住了在gazebo里面移动机器人模型，导致建模稍微有些偏差)
                    * world6（13min）
                        * ![](https://notes.sjtu.edu.cn/uploads/upload_d5144ef099fe97beb8be793fb5b28542.png)
                    * world5（10min）
                        * ![](https://notes.sjtu.edu.cn/uploads/upload_6f1769a9ecea5f31d287d9c86dc553b7.png)


                * rqt_graph
                    * ![](https://notes.sjtu.edu.cn/uploads/upload_904951cfa00614b66691642f07322312.png)

            * 方案2：使用实验室第二套局部避障算法进行路径规划
                * 配置环境
                * 编译
               ``` catkin build ```
                * 启动gazebo仿真场景
                ```roslaunch ranger_mini_gazebo ai_robot_ranger_gazebo.launch```
                * 启动局部避障
                ```roslaunch ai_robot_nav_demo obs_avoid.launch```
                * 启动速度平滑
                ```roslaunch yocs_velocity_smoother standalone.launch```
                * 进行自身定位及全局路径规划与gmapping建图
                ```roslaunch ai_robot_nav_demo rrt_slam.launch ```
                * 启动rrt_exploration探索
               ``` roslaunch rrt_exploration simple.launch```
                * 保存探索地图
                ```roslaunch ai_robot_nav_demo save_map_rrt.launch```
                * 正在建图
                    * ![](https://notes.sjtu.edu.cn/uploads/upload_053f490deaaf8b4a7aa15c237cb23797.png)
                * 自主探索生成地图
                    * world6(18min)
                        * ![](https://notes.sjtu.edu.cn/uploads/upload_ac59e7067c7e842c75ef3dbf46cdc8b7.png)
                    * world5（15min）
                        * ![](https://notes.sjtu.edu.cn/uploads/upload_ee2d6528154c1d89d2e891e043cc5a5f.png)

                * rqt_graph
                    * ![](https://notes.sjtu.edu.cn/uploads/upload_636f56b012d70dc1d36df0e78a12e7ac.png)

                * 问题
                    * 针对不同的地图需要更改参数，适应性差
                    * 明明还有没有探索的部分却不生成探索路径
                        * 可以尝试使用A*取代Dijkstra（已做）
                    * 计算前沿信息增益花费时间过长
                    * 如果向一个方向即将（还没）探索完（这时它的信息增益I很小了），还是可能会被其他位置的目标边界点吸引，这样会增加重复性和探索时间
                * 思考
                    * 参数可以通过机器学习的聚类和分类方法设置
                    * 考虑和explore_lite结合

* 评价部分
    * 效率（在时间、燃料约束条件下达到最高的地图探索率的规划问题）
        * 时间性能：固定要求的地图探索率，比较各个探索算法实现的时间（大致看）
            * explore_lite>rrt_exploration
        * 平均地图探索率：固定探索的时间，比较固定时间内的地图探索率（比较固定时间内的地图完整度）
            * 复杂地图rrt_exploration>explore_lite
            * 简单地图explore_lite>rrt_exploration
        * 路径最优性：通过rviz显示建图的轨迹，查看重复探索路径与高效探索路径的长度（待完成）
    * 环境适应性：采用不同特点的仿真环境，考察各个算法的探索性能（比较相同算法在不同复杂度环境下的适用性）
        * world5：多封闭角落，复杂
        * world6：较为简单
        * explore_lite>rrt_exploration
* 参考资料
    * 实验室提供的导航模块
        * https://github.com/546454596/ai_robot_practice
    * 一个rrt_exploration的优秀例子
        * https://github.com/fazildgr8/ros_autonomous_slam
    * 三个探索算法的总结
        * https://blog.csdn.net/qq_52785580/article/details/131060515
    * 一个ros的非常详细的教程
        * http://www.autolabor.com.cn/book/ROSTutorials/
    * actionlib教程（填补上面教程缺少的部分）
        * https://blog.csdn.net/u011118482/article/details/72989161
    * rrt_exploration在ubuntu20.04+ros noetic可用的教程
        * https://bingda.yuque.com/staff-hckvzc/ai5gkn/cpf18i
    * 路径规划
        * https://zhuanlan.zhihu.com/p/54510444
    * teb
        * https://www.leiphone.com/category/transportation/0TCtaBOIcFOIBN69.html
    * dwa
        * https://blog.csdn.net/weixin_44504228/article/details/115698377?spm=1001.2014.3001.5501
    * 局部规划对比
        * https://blog.csdn.net/lovely_yoshino/article/details/117669476
    * 机器学习、图论等模型相关知识（交大数学建模营资料库）
        * https://anl.sjtu.edu.cn/mcm/docs/name
————————————————
## 暑期科研实习心得：）
1. 谨慎更新已经配置好的包
