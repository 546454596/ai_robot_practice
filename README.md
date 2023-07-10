# AI Robot Ranger Laser

**安装与编译：**

1.新建工作空间，如:

```
mkdir -p ai_robot_ranger_mini/src
```

2.将ai_robot_laser_core.zip放到src目录下，并解压缩

3.对于版本管理的包’ai_robot_ranger_laser’，在src目录下通过git clone下载：

```
git clone git@github.com:546454596/ai_robot_practice.git
```

4.安装工作空间中包的依赖，首先切换到工作空间下：

```
cd ai_robot_ranger_mini
```

5.运行下述命令安装该工作空间的所有依赖（如果没有rosdep，可参考http://wiki.ros.org/rosdep 安装）：

```
rosdep install --from-paths src --ignore-src -r -y
```

6.进行编译：

```
catkin build
```

**基于拓扑地图和开发的导航模块的使用流程**

1.打开仿真场景

```
roslaunch ranger_mini_gazebo ai_robot_ranger_gazebo.launch
```

**注意**：通过在ai_robot_ranger_gazebo.launch文件中，修改下面**world0**即可更换仿真场景，world0是根据实验室场景构建，本次提供的版本只有world0下的拓扑地图，暂时还不支持其他word下的导航。但探索建图不实用图谱地图，所以在复现**explore_lite** 时，可以自由修改仿真world。 

`<include file="$(find gazebo_ros)/launch/empty_world.launch">

​        <arg name="world_name" value="$(find ranger_mini_gazebo)/worlds/$(arg world_0)" />

​    </include>`

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
roslaunch ai_robot_navigation obsavoid.launch sim:=true
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


