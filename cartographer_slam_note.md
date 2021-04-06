# note for cartographer



## 1、install 

you can run next cmd,



~~~shell
sudo apt-get install ros-melodic-cartographer-ros
~~~

but this version of cartographer is 1.0.0

you should git cartographer2.0.0 from github, then build it manually.

see :https://blog.csdn.net/xiaokai1999/article/details/112791787?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.control&dist_request_id=1328697.582.16166661116823559&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.control

of course , 

you can copy cartographer2.0.0 file that haved build on other pc to your own pc.

then, source your package path.



## 2、pure localization 

see file : [demo_backpack_2d_localization.launch](../../catkin_new/install_isolated/share/cartographer_ros/launch/demo_backpack_2d_localization.launch) 

<launch>
  <param name="/use_sim_time" value="true" />

  <param name="robot_description"
    ==textfile="$(find cartographer_ros)/urdf/backpack_2d.urdf" />==

  <node name="robot_state_publisher" pkg="robot_state_publisher"
    type="robot_state_publisher" />

  <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
         ==-configuration_basename backpack_2d_localization.lua==

​          -load_state_filename $(arg load_state_filename)"
​      output="screen">
​    <remap from="echoes" to="horizontal_laser_2d" />
  </node

  <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros"
      type="cartographer_occupancy_grid_node" args="-resolution 0.05" />

  <node name="rviz" pkg="rviz" type="rviz" required="true"
      ==args="-d $(find cartographer_ros)/configuration_files/demo_2d.rviz" />==
  <node name="playbag" pkg="rosbag" type="play"
      args="--clock $(arg bag_filename)" />
</launch>

you should sure UDRF、LUA、RVIZ file is OK。

notice：

LUA file can contain other LUA file, so you should check all file that used.






