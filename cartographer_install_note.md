
one way:
if you haved installed ROS melodic,just run next cmd(cartographer-ros version is 1.0.0);
$ sudo apt install ros-melodic-cartographer-ros

two way:

reference: https://www.cnblogs.com/hitcm/p/5939507.html

向大神致敬,第一种国内大神修改版如下:

1.install dependency

$  sudo apt-get install -y google-mock libboost-all-dev  libeigen3-dev libgflags-dev libgoogle-glog-dev liblua5.2-dev libprotobuf-dev  libsuitesparse-dev libwebp-dev ninja-build protobuf-compiler python-sphinx  ros-kinetic(特别注意这个和你安装的ros版本相匹配,选择适合自己的)-tf2-eigen libatlas-base-dev libsuitesparse-dev liblapack-dev

2.install ceres solver (路径你可以自己随意,不过建议最好和和下面安装的放在一起;如: path:/home/zc/)

$  git clone https://github.com/hitcm/ceres-solver-1.11.0.git
$  mkdir build
$  cd ceres-solver-1.11.0/build
$  cmake ..
$  make –j
$  sudo make install
(err:check dependency 如果报错,请查看依赖项是否添加正确)

3.install cartographer

$  git clone https://github.com/hitcm/cartographer.git
$  mkdir build
$  cd cartographer/build
$  cmake .. -G Ninja
$  ninja
$  ninja test
$  sudo ninja install

4.install cartographer_ros

$  sudo apt-get update
$  sudo apt-get install -y python-wstool python-rosdep ninja-build
$  mkdir catkin_ws
$  cd catkin_ws
$  wstool init src
$  cd src
$  git clone https://github.com/hitcm/cartographer_ros.git
$  cd
$  cd catkin_ws   
$  catkin_make // or catkin_make_isolated

最后为了避免每次都要加工作空间麻烦,所以可以执行下面的直接写好即可.
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

or

echo "source /home/l/catkin_ws/devel_isolated/setup.bash" >> ~/.bashrc

到此国内大神修改版安装完成可以下载demo自己玩了,不过对电脑性能要求还是有点高的,通过建图你就可以看出来了,这里不啰嗦了.
————————————————
版权声明：本文为CSDN博主「zchao9456」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/mylovechao/article/details/83902951
