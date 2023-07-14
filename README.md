# AI Robot Ranger Laser

**安装与编译：**

1.新建工作空间，如:

```
mkdir -p ai_robot_ranger_mini/src
```

2.进入src目录，并git clone **ai_robot_practice_1**（该包主要用于构建拓扑地图、实现导航等）：

```
cd ai_robot_ranger_mini/src
git clone -b dev git@github.com:546454596/ai_robot_practice_1.git
```

3.同样在src目录下，git clone **ai_robot_ranger_laser**（该包主要提供仿真场景、导航demo的launch文件、魔改的move_base等）

```
git clone git@github.com:546454596/ai_robot_practice.git
```

4.安装工作空间中包的依赖：

（1）首先切换到工作空间下，

```
cd ../
```

（2）然后运行下述命令安装该工作空间的所有依赖（如果没有rosdep，可参考http://wiki.ros.org/rosdep 安装）：

```
rosdep install --from-paths src --ignore-src -r -y
```

5.进行编译（注意需要提前安装catkin-tools,参考https://blog.csdn.net/dzhongjie/article/details/83868684），然后激活工作空间：

```
catkin build
source devel/setup.bash
```

**基于拓扑地图和开发的导航模块的使用流程**
一键启动所有launch：
```
roslaunch ai_robot_bringup start_locnav_amcl_sim.launch
```
逐步启动如下：

1.打开仿真场景

```
roslaunch ranger_mini_gazebo ai_robot_ranger_gazebo.launch
```

**注意**：通过在ai_robot_ranger_gazebo.launch文件中，修改下面**world0**即可更换仿真场景，world0是根据实验室场景构建，本次提供的版本只有world0下的拓扑地图，暂时还不支持其他word下的导航。但探索建图不使用拓扑地图，所以在复现**explore_lite** 时，可以自由修改仿真world。 
```
<include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find ranger_mini_gazebo)/worlds/$(arg world_0)" />
</include>
```

2.载入地图、启动定位：

```
roslaunch ai_robot_nav_demo brainnav.launch 
```

3.启动全局规划

```
roslaunch ai_robot_navigation globalnav_sim.launch
```

4.启动局部规划

```
roslaunch ai_robot_navigation obsavoid_sim.launch
```

**基于拓扑地图和TEB局部避障算法的使用流程**

1.打开仿真场景：

```
roslaunch ranger_mini_gazebo ai_robot_ranger_gazebo.launch
```

2.载入地图、启动定位、启动全局路径规划：

```
roslaunch ai_robot_nav_demo brainnav.launch move_base:=true
```

3.启动局部避障：

```
roslaunch ai_robot_nav_demo obs_avoid.launch
```

4.启动速度平滑：

```
roslaunch yocs_velocity_smoother standalone.launch
```

