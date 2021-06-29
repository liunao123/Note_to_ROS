# ROS Note



<img src="/home/xiaofan/Desktop/Note/sendpix0.jpg" style="zoom:50%;" />

## catkin

Ros的编译工具,基于==cmake==实现

~~~bash
cd ~catkin_ws //一定要在工程目录下进行编译
catkin_make
source ***
~~~



## package

package是catkin编译的基本单元

package是catkin编译的对象,对其进行递归编译

包括==CMakeLists.txt==和==package.xml==两个文件

前者是**编译过程**,后者指定***属性描述***

==package是ROS的基本组织形式,,,一个package可以包含多个EXE()节点==

<img src="/home/xiaofan/.config/tencent-qq//AppData/file//sendpix5.jpg" style="zoom:80%;" />





## 常用命令

1. rospack

   查找pkg的地址

   ~~~sh
   rospack list
   ~~~

2. roscd

   跳转到某个pkg下

   ~~~
   roscd pkg_name
   ~~~

   

3. rosls

   列举某个pkg下的文件信息

4. rosed-(==edit==)

   编辑pkg中的文件

5. catkin_create_pkg

   创建一个pkg

   ~~~sh
   catkin_create_pkg <pkg_name> [deps]
   ~~~

   

6. rosdep

   安装某个pkg所需的依赖

   ~~~sh
   rosdep install [pkg_name]
   ~~~


7. rosbag

   ~~~
   rosbag info bagname.bag
   
   
   播放一个包，用rviz进行可视化
   roscore
   rosbag play bagname.bag
   rosrun rviz rviz
   
   
   rosbag record -a //订阅所有topic，重新写到一个bag文件里
   
    //订阅某些topic，记录300s的时间
   rosbag record /sacn /laser /imu --duration 300
   
   ~~~

   

8. 

9. ACTION

10. ~~~bash
    
    
    ~~~
    
    

   ~~~
   
   
   ~~~

   

11. 官方文档的笔记

    ~~~
    rosmsg show geometry_msgs/Pose
    rosrun rqt_plot rqt_plot
    
    rostopic hz /turtle1/pose
    rostopic info  /turtle1/cmd_vel
    rostopic pub  -r 3  /turtle1/cmd_vel geometry_msgs/Twist -- '[2.10, 0.1, 0.0]' '[0.0, 0.0, 3.28]'
    rostopic type /turtle1/cmd_vel
    rosrun turtlesim turtlesim_node __name:=m
    ~~~

    

12. rqt轨迹可视化工具

~~~
roslaunch rqt_joint_trajectory_plot generator.launch
roslaunch rqt_joint_trajectory_plot rqt_plot_trajectory_pva.launch
rosbag play '/home/l/Desktop/run_cartographer_on_robot/2021-03-24-14-26-56.bag' 
rqt --force-discover

~~~



## TF

~~~
rqt
//直接的得到 tf tree
rosrun tf tf_echo /tf1 /tf2
//得到tf2相对于tf1<以 tf1 为坐标原点>的变换关系，
//得到的值等于  tf2 - tf1，

/<target_frame>相对于<source_frame>的变换（坐标及旋转）
rosrun tf tf_monitor <source_frame> <target_frame> 
rosrun tf tf_monitor world turtle1
rosrun tf_echo <source_frame> <target_frame> 
rosrun tf tf_echo world turtle1

~~~







## odom messgae

![image-20210514153938838](/home/l/.config/Typora/typora-user-images/image-20210514153938838.png)

odom中，pose的数据以odom为frame

twist数据以child_frame_id为坐标系





## rosbag filter

## 将特定topic、特定tf tree上的某些connection 记录下来，

~~~
rosbag filter input.bag output22.bag " topic == '/scan' or topic == '/odom' or topic == '/tf' and  m.transforms[0].header.frame_id != '/map' and m.transforms[0].child_frame_id != 'odom' "


rosbag filter in.bag out.bag " topic == '/imu_50' or topic == '/scan' or topic == '/odom' or topic == '/tf' and  m.transforms[0].header.frame_id != '/map' and m.transforms[0].child_frame_id != 'odom' and  m.transforms[0].header.frame_id != '/odom' and m.transforms[0].child_frame_id != 'base_link'"

rosbag filter t.bag tout.bag "topic == '/imu' or topic == '/scan' or topic == '/odom' or topic == '/tf' and  m.transforms[0].header.frame_id != '/map' and m.transforms[0].child_frame_id != 'odom' and  m.transforms[0].header.frame_id != '/odom' and m.transforms[0].child_frame_id != 'base_link'"



截取一段时间内的
rosbag filter '/home/l/Desktop/run_cartographer_on_robot/20210611/2021-06-11-huashi/2021-06-11-15-28-46.bag'  tout.bag " t.to_sec() >= 1623396526.00 and t.to_sec() <= 1623397056.00 "
~~~

例子，去除map-odom，和odom-base_link的tf

<img src="/home/l/Desktop/Note/Note_to_ROS/frames.png" alt="frames" style="zoom:80%;" />



## rosbag extract sth to txt

rostopic echo -b <BAGFILE> -p <TOPIC> > <output>.txt