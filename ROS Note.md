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

   