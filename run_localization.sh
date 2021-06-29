#建图
#roslaunch cartographer_ros library.launch

#roslaunch cartographer_ros library_offline_backpack_2d.launch bag_filenames:='/home/xiaofan/Desktop/run_cartographer/pure_locate/20210427/2021-04-27-17-15-32.bag'


roslaunch cartographer_ros library_backpack_2d_localization.launch load_state_filename:='/home/l/Desktop/run_cartographer_on_robot/pure_locate/20210427/2021-04-27-17-15-32.bag.pbstream'   bag_filename:=/home/l/Desktop/run_cartographer_on_robot/pure_locate/20210427/2021-04-27-14-51-47.bag


#gnome-terminal --tab -x bash -c "rosrun rosbag play 2021-04-13-18-09-24.bag ;exit "
#gnome-terminal --tab -x bash -c "rosrun beginner_evo csv.py  ;exit "

#roslaunch cartographer_ros library.launch

roslaunch cartographer_ros library_offline_backpack_2d.launch bag_filenames:=


#定位
roslaunch cartographer_ros library_backpack_2d_localization.launch load_state_filename:=/home/xiaofan/Desktop/run_cartographer/pure_locate/20210428/2021-04-28-17-10-42-out-out.bag.pbstream  bag_filename:=/home/xiaofan/Desktop/run_cartographer/pure_locate/2021-04-13-18-09-24.bag
roslaunch cartographer_ros library_backpack_2d_localization.launch load_state_filename:='/home/l/Desktop/run_cartographer_on_robot/pure_locate/2021-04-06-20-29-53.bag.pbstream'



#roslaunch cartographer_ros visualize_pbstream.launch pbstream_filename:='/home/l/Desktop/run_cartographer_on_robot/pure_locate/20210427/2021-04-27-17-15-32.bag.pbstream'

#查看pb
roslaunch cartographer_ros visualize_pbstream.launch pbstream_filename:=

rosbag filter 2021-06-07-19-46-09.bag 2021-06-07-19-46-09-tout.bag "topic == '/joy' or topic == '/joy/cmd_vel' or topic == '/imu' or topic == '/scan' or topic == '/odom' or topic == '/tf_static' or topic == '/tf' and  m.transforms[0].header.frame_id != '/map' and m.transforms[0].child_frame_id != 'odom' and  m.transforms[0].header.frame_id != '/odom' and m.transforms[0].child_frame_id != 'base_link'"



##### rosbag 
rosbag filter 2021-06-07-19-46-09.bag 2021-06-07-19-46-09-tout.bag "topic == '/joy' or topic == '/joy/cmd_vel' or topic == '/imu' or topic == '/scan' or topic == '/odom' or topic == '/tf_static' or topic == '/tf' and  m.transforms[0].header.frame_id != '/map' and m.transforms[0].child_frame_id != 'odom' "




rosbag filter tout.bag tout-350.bag "t.to_sec() >= 1624343370 and t.to_sec() <= 1624343660 "


rosbag filter 2021-06-22-13-12-33.bag tout.bag "topic == '/scan' or topic == '/odom' or topic == '/tf' and  m.transforms[0].header.frame_id != '/map' and m.transforms[0].child_frame_id != 'odom' "


rosbag filter 2021-06-22-14-29-13.bag tout.bag "topic == '/scan' or topic == '/odom' or topic == '/tf' and  m.transforms[0].header.frame_id != '/odom' and m.transforms[0].child_frame_id != 'base_link' "

 python '/home/xiaofan/catkin_gy/src/robot_localization_tools/scripts/remove_tf.py'

#保存ROS地图
#rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=./mymap -pbstream_filename=2021-04-06-20-27-06.bag.pbstream -resolution=0.05

1 完成轨迹，不接受下一步数据

$ rosservice call /finish_trajectory 0
2 序列化保存当前状态（地图保存为.pbstream形式）

$ rosservice call /write_state "{filename: '${HOME}/Downloads/mymap.pbstream'}"
3 将.pbstream形式地图转化成.pgm和.yaml（ROS标准形式地图）

利用 cartographer_pbstream_to_ros_map 节点
$ rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/Downloads/mymap -pbstream_filename=${HOME}/Downloads/mymap.pbstream -resolution=0.05

# rosrun cartographer_ros cartographer_pbstream_to_ros_map -resolution=0.05 -map_filestem=0.25 -pbstream_filename=1.bag.pbstream

############3  evo_traj  

#evo_traj bag '/home/xiaofan/Desktop/run_cartographer/pure_locate/20210428/2021-04-28-17-16-44.bag'  /odom -p
#evo_traj tum *.txt --plot --plot_mode=xy
#python3 '/home/l/catkin_ws/src/robot_localization_3/script/csv_py3_trajectory.py'




