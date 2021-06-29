# library robot Note

1. ## TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 16

   Number of range data to accumulate into one unwarped, combined range data to use for scan matching.
   This variable defines the number of messages required to construct a full scan (typically, a full revolution)

   该参数在  10-25   之间，机器人位置在初始化最快。在旋转180°左右时，即可实现定位。
   当旋转角度较小时，以较慢的旋转速度进行旋转，大部分位置也可以实现定位，只不过定位时间稍长。

   

2. ## POSE_GRAPH.constraint_builder.global_localization_min_score = 0.5

   该值在0.55--0.65比较好
   较大的值，定位精度高，但是反应慢，反之则反之

   

3. ## POSE_GRAPH.optimize_every_n_nodes = 0

   设置为0，即：关闭全局SLAM

   

4. ## TRAJECTORY_BUILDER_2D.submaps.num_range_data = 15 

   定位时，Trajectory1的第一个数，为这个数的  2% （是不是和其他参数有关，尚不清楚）

5. 

6. 

7. 

8. ## temp

9. 



## 










## 1、SLAM定位精度的自我评估(具体使用尚不明确)

link:https://google-cartographer.readthedocs.io/en/latest/evaluation.html

使用以下命令：

./cartographer_autogenerate_ground_truth -pose_graph_filename ==optimized.bag.pbstream== -output_filename ==relations.pbstream== -min_covered_distance 100 -outlier_threshold_meters 0.15 -outlier_threshold_radians 0.02

./cartographer_compute_relations_metrics -relations_filename ==relations.pbstream== -pose_graph_filename ==test.pbstream==

## 2、定位完成后的优化

cartographer在定位结束后，会对轨迹以及定位进行优化，不过较慢，

在定位的时候，==车速应该慢一些==

该过程涉及==constraint_builder_2d.cc==，

下次了解该函数所涉及的参数，进行修改。



## 3、flag of localization is successful or not

1. if cartographer has initially localized wrong, you do want it to make the jump and relocalize
2. cartographer already has that functionality and "jump" thresholds can be set from config files

However, there are a few solutions:

1. use a locally accurate sensor (odometry) and tune trust values in cartographer
2. you can use a global visual odometry-based localizer that can determine the "region" in which you should be



## 4、指定初始位置

### Services

start_trajectory ([cartographer_ros_msgs/StartTrajectory](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/StartTrajectory.srv))

## [cartographer_ros](https://github.com/cartographer-project/cartographer_ros)/[cartographer_ros_msgs](https://github.com/cartographer-project/cartographer_ros/tree/master/cartographer_ros_msgs)/[srv](https://github.com/cartographer-project/cartographer_ros/tree/master/cartographer_ros_msgs/srv)/**StartTrajectory.srv**

Starts a trajectory using default sensor topics and the provided configuration. An initial pose can be optionally specified. Returns an assigned trajectory ID.



== https://github.com/cartographer-project/cartographer_ros/pull/1284 ==

https://github.com/cartographer-project/cartographer_ros/blob/5b09b08daf6a6f1f51296f44e11f8c1f5ba48d40/cartographer_ros/cartographer_ros/set_initpose_main.cc#L109



## EVO使用记录

1. 把两个轨迹一起画出来
2. evo_traj tum ==2021_04_20_13_25_12_tum.txt==  ==2021_04_20_13_21_04_tum.txt== --ref=2021_04_20_13_25_12_tum.txt == --plot --plot_mode=xy



NOTE:

1、纯定位选项在trajectory_builder.lua文件中,（--version 1.0.0）
    pure_localization = true,

2、
 在trajectory_builder_2d.lua参数文件中，
    use_imu_data = false,（无 IMU data）

3、建立地图时，保存成pbstream文件。
当使用cartographer在线建图时，cartographer不知道什么时候结束，所以要先调用服务来关闭传感器数据的接收。这个命令执行完，根据你的建图规模稍微等下，有的场景太大可能后端还没完成，然后你可以在RVIZ看到地图的样子了。



```text
# 先停止建图：
rosservice call /finish_trajectory 0
# 然后保存成pbstream，自己更改保存路径，注意不能写相对路径，测试过不行
rosservice call /write_state "{filename: '${HOME}/Downloads/m.pbstream'}"
#导出成一般导航使用的yaml+pgm
rosrun cartographer_ros cartographer_pbstream_to_ros_map -resolution=0.05 -map_filestem=mapname -pbstream_filename=pbstream_name
# 保存可能有更方便的操作，我这边参考的是：
```



4、这里使用了之前博客的lua文件和urdf文件，已经建好的地图pbstream文件。需要自行查询，也可以include你的lua，或者设置成你的urdf文件，还有建图和定位的分辨率必须一致，这里都为0.05。

5、需要修改/cartographer_ros/cartographer_ros/occupancy_grid_node_main.cc 的源码，在main函数上面3行作用，注释掉下面这句。原因是因为他会发布新的地图覆盖原来的地图，建图的时候请记得打开奥。


6、Published Topics
map (nav_msgs/OccupancyGrid)
The published occupancy grid topic is latched（锁住的）.

ROS的topic一般是实时的，现在的节点无法获取以前节点发布的topic
一旦，topic是latched，，表示：当前的节点所发布的topic，在一段时间后，其他node依然可以接收到。

7、TF tree

首先是两个开关参数`provide_odom_frame`和`use_odometry`，`use_odometry`是控制是否使用里程计信息，`provide_odom_frame`是控制carto是否提供里程计tf。

- `provide_odom_frame`开关参数：这里先理解carto建图时一定会有的`map_frame`也就是map的tf，建图时carto本质上能得到的一般是base_link与map之间的位置关系（因为激光雷达和imu的tf与base_link一般是固定的，map即它建得的地图固定坐标系），而描述位置关系在ROS一般就直接是两个坐标系的tf，但是我们知道tf树是不能多个导向同一个的，会冲突，所以如果本身有坐标系指向base_link（比如odom、base_footprint这些），那么map要描述与base_link的位置关系就只能连到再上一层了（比如map→odom→base_link就可以）。了解了这个基本机制之后，就可以来了解另外两个配置参数怎么一起使用了，另外的两个配置参数只与这个开关参数有关，前面说了carto建图知道的是map与base_link之间的关系，但是由于tf树的机制并不是任意情况都能够直接用tf连接base_link与map的，而carto内部不知道你外部tf树的复杂情况，因此提供了这两个参数要求你手动定义，首先说明`published_frame`参数，这个参数理解为指定你要把map连接到哪个坐标系（因为不一定能够直接连base_link），假如base_link上面本身你已经有odom的tf连接着，那么这个`published_frame`就可以指定为`odom`，如果只是这样普通的情况，直接就这样设置，然后设置`provide_odom_frame`为false即可，即不需要carto提供odom的tf，因为我们本身有了，这种情况下map就会直接连接`published_frame`，==而假如我们没有odom的tf，而且我们希望carto能够给我们提供一个odom的tf（因为其实carto得到了这个信息），那么就将`provide_odom_frame`设置为true，同时`odom_frame`设置为你想要carto提供的odom的tf名字，这时，carto就会帮你生成`map→odom_frame→publisher_frame`这样的tf，对于这种我们没有odom的情况，一般`published_frame`也就指定为base_link或者base_footprint==。
- `use_odometry`开关参数：这个直接决定是否使用里程计信息（一般也就是我们是否有提供里程计信息），还是需要先理解一个前置知识，就是odom的tf与odom话题之间的关系，两者有关系，但并不等同，odom的tf一般也就是指odom与base_link的坐标系关系，它本身只有坐标系关系，即两个坐标系之间的平移矩阵、旋转矩阵；而odom话题就需要了解它的话题类型了，odom即里程计话题一般有专门的类型`nav_msgs/Odometry`，它内部不仅仅有tf包括的信息（它会指定child_frame的名字然后包括了旋转、平移的关系，具体去搜这个话题的定义），还会有速度信息、协方差矩阵也就是不确定性关系这些，因此如果odom的tf需要转换成odom的话题，需要手动定义协方差以及速度。了解了这些后，回到`use_odometry`的意义，它本身是决定是否使用里程计的信息，现在可以进一步说明是决定是否使用odom的话题，它一定是使用odom的话题，因此launch文件那个odom的remap就是和这个一起联动使用的。`odom_frame`和`published_frame`参数与`use_odometry`无关。

8、library robot 开机
ssh sage@192.168.1.6
进入远程，机器人系统

then
roslaunch sage_main main.launch





rosrun rqt_plot rqt_plot /imu/linear_acceleration/x:y:z

 1991  rqt_plot /odom/twist/twist/angular/x:y:z
 1992  rqt_plot /odom/pose/pose/position/x:y:z



## offline mapping

roslaunch cartographer_ros library_offline_node.launch bag_filenames:= bagname





# cartographer code

link;https://gaoyichao.com/Xiaotu/?book=Cartographer%E6%BA%90%E7%A0%81%E8%A7%A3%E8%AF%BB&title=index

==测试==
如果在运行cartographer_node的时候通过命令行参数load_state_filename指定了保存有SLAM状态的文件，那么就加载该状态文件继续运行。

![image-20210428162905743](/home/l/.config/Typora/typora-user-images/image-20210428162905743.png)



20210428-pm-17:00

https://gaoyichao.com/Xiaotu/?book=Cartographer%E6%BA%90%E7%A0%81%E8%A7%A3%E8%AF%BB&title=cartographer_node%E7%9A%84%E5%A4%96%E5%A2%99_node%E5%AF%B9%E8%B1%A1



