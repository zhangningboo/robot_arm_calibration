# ROS melodic 安装
系统采用 [ubuntu 18.04](https://releases.ubuntu.com/18.04/)
#### 清华源配置 
```shell
$ sudo apt update
$ sudo apt install -y vim
$ sudo vim /etc/apt/sources.list
# 默认注释了源码镜像以提高 apt update 速度，如有需要可自行取消注释
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb http://security.ubuntu.com/ubuntu/ bionic-security main restricted universe multiverse
:wq
$ sudo apt update && sudo apt upgrade
```
#### 设置sudoer用户免密码
```shell
# 取消密码
$ sudo vim /etc/sudoers
# 添加内容：
ubuntu ALL=(ALL) NOPASSWD:ALL
```
#### 安装openssh-server并配置互信
```shell
$ sudo apt install -y openssh-server
$ sudo /etc/init.d/ssh start
$ mkdir  ~/.ssh && touch ~/.ssh/authorized_keys
# 把主机的id_rsa.pub拷贝到authorized_keys
```
#### vscode ssh连接并安装插件
- ROS
- catkin-tools
- XML
- CMake
- C/C++
- Python

#### ROS melodic 安装
```shell
$ sudo vim /etc/apt/sources.list.d/ros-latest.list
# 添加内容：
deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ bionic main
:wq
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install -y python-pip
$ python -m pip install --upgrade pip -i https://pypi.tuna.tsinghua.edu.cn/simple
$ python3 -m pip install --upgrade pip -i https://pypi.tuna.tsinghua.edu.cn/simple
$ pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

$ sudo apt install -y ros-melodic-desktop-full
$ sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
$ sudo pip install rosdepc
$ sudo rosdepc init
$ rosdepc update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && source ~/.bashrc
$ sudo apt install -y ros-melodic-rosparam-shortcuts
$ cd ~ && mkdir workspace && cd ~/workspace && git clone https://github.com/gflags/gflags.git
$ cd gflags && mkdir build && cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON - DGFLAGS_ NAMESPACE=gflags ..
$ make -j4 && sudo make install
$ sudo apt install -y ros-melodic-ros-control ros-melodic-ros-controllers
$ mkdir -p ~/workspace/catkin_ws/src && cd ~/workspace/catkin_ws && catkin_make
$ source ~/workspace/catkin_ws/devel/setup.bash
```

#### 安装MoveIt

```shell
$ sudo apt install -y python-catkin-tools clang-format-3.9
$ sudo apt install -y ros-melodic-moveit \
                      ros-melodic-joint-state-publisher-gui \
                      ros-melodic-moveit-resources \
                      ros-melodic-moveit-visual-tools \
                      ros-melodic-code-coverage \
                      ros-melodic-franka-description \
                      ros-melodic-joy \
                      ros-melodic-canopen-motor-node \
                      ros-melodic-panda-moveit-config \
                      ros-melodic-ecl* \
                      ros-melodic-serial* \
                      ros-melodic-joint-state* \
                      ros-melodic-handeye \
                      
$ mkdir -p ~/workspace/moveit_ws/src && cd ~/workspace/moveit_ws/src
$ git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel
$ sudo rosdep fix-permissions && rosdep install -y --from-paths . --ignore-src --rosdistro melodic
$ cd ~/workspace/moveit_ws && catkin_make
$ sudo apt install -y ros-melodic-moveit ros-melodic-moveit-pr2
$ roslaunch moveit_setup_assistant setup_assistant.launch
```

#### 安装easy_handeye

```shell
$ sudo -H /usr/bin/python2 -m pip install -U transforms3d==0.3.1
$ sudo apt install -y ros-melodic-freenect* ros-melodic-aruco*  ros-melodic-visp*
$ mkdir -p ~/workspace/calibration_ws/src && cd ~/workspace/calibration_ws/src
$ git clone https://github.com/IFL-CAMP/easy_handeye.git -b v0.4.3
$ cd ~/workspace/calibration_ws && catkin_make
$ pip install opencv-python==4.1.2.30 opencv-contrib-python==4.1.2.30
$ roslaunch easy_handeye eye_in_hand_calibration.launch
$ 
```

#### 安装相机
- [OpenNI2 SDK](https://developer.orbbec.com.cn/develop_details.html?id=2)
- [奥比中光 ROS1驱动](https://github.com/orbbec/OrbbecSDK_ROS1)
- 使用奥比中光系列相机，本步骤可省略，直接使用本工程里的版本
```shell
$ sudo apt install -y build-essential freeglut3 freeglut3-dev
$ sudo apt install -y libgflags-dev \
    ros-$ROS_DISTRO-image-geometry \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-publisher \
    libgoogle-glog-dev \
    libusb-1.0-0-dev \
    libeigen3-dev
$ mkdir -p ~/workspace/calibration_ws/src && cd ~/workspace/calibration_ws/src
$ git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
$ cd ~/workspace/calibration_ws && catkin_make

# USB规则
$ source ./devel/setup.bash
$ roscd orbbec_camera && cd script
$ sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules
$ sudo udevadm control --reload && sudo  udevadm trigger

$ sudo apt install -y ros-melodic-rqt ros-melodic-rqt-common-plugins
# gemini.launch 可以换成实际使用的设备型号
$ roslaunch orbbec_camera gemini.launch

# 创建新terminal
$ rqt_image_view

$ rostopic list
$ rosservice list
$ rosparam list
```
#### 机械臂
- [机械臂在线文档](https://fr-documentation.readthedocs.io/zh-cn/latest/)
- [机械臂关节DH参数](https://fr-documentation.readthedocs.io/zh-cn/latest/CobotsManual/robot_brief_introduction.html#id4)
- 注意修改机械臂的`IP`地址
- 法奥官方版本：
```shell
$ mkdir -p ~/workspace/calibration_ws/src && cd ~/workspace/calibration_ws/src
$ git clone https://github.com/FAIR-INNOVATION/frcobot_ros.git
$ sudo apt install -y \
    ros-melodic-rosparam-shortcuts \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-moveit \
    ros-melodic-xmlrpcpp \
    ros-melodic-xmlrpcpp-dbgsym \
    libxmlrpcpp-dev \
    libxmlrpcpp1d
$ cd ~/workspace/calibration_ws && catkin_make
# 修改 frcobot_hw.launch文件中机器人的ip地址
# 注意！rvs_ip修改为ROS的主机ip
$ sudo cp -R ~/workspace/calibration_ws/src/frcobot_ros/frcobot_hw/include/* /usr/local/include/
$ sudo cp -R ~/workspace/calibration_ws/src/frcobot_ros/frcobot_hw/lib/* /usr/local/lib/
$ sudo cp -R ~/workspace/calibration_ws/src/frcobot_ros/frcobot_gripper/include/* /usr/local/include/
$ sudo cp -R ~/workspace/calibration_ws/src/frcobot_ros/frcobot_gripper/lib/* /usr/local/lib/
$ sudo cp -R ~/workspace/calibration_ws/src/frcobot_ros/frcobot_camera/include/* /usr/local/include/
$ sudo cp -R ~/workspace/calibration_ws/src/frcobot_ros/frcobot_camera/lib/* /usr/local/lib/
$ roslaunch frcobot_hw frcobot_hw.launch

$ rostopic echo /frcobot_status
```

- 本教程版本：
```shell
$ sudo cp -R ~/workspace/calibration_ws/src/fr5_real_moveit/include/* /usr/local/include/
$ sudo cp -R ~/workspace/calibration_ws/src/fr5_real_moveit/lib/* /usr/local/lib/
$ roslaunch fr5_real_moveit fr5robot_rviz.launch
```

#### 标定步骤
- 打印ArUco码
- 启动机械臂
- 启动相机
- 启动标定

```shell
$ roslaunch fr5_real_moveit demo.launch
$ roslaunch orbbec_camera gemini.launch
$ roslaunch orbbec_camera gemini_info_node
$ roslaunch easy_handeye eye_in_hand_calibration.launch
$ roslaunch easy_handeye start_calibrate.launch
```

Computed calibration: [[-0.04291248  0.62296193 -0.78107423 -0.0528288 ]
 [ 0.02377933  0.78220978  0.62256117  0.06588526]
 [ 0.99879581  0.00814223 -0.04838018  0.04474217]
 [ 0.          0.          0.          1.        ]]
 /home/ubuntu/.ros/easy_handeye/fr5_orbbec_hand_eye_calibration_eye_on_hand.yaml
https://blog.csdn.net/laoli_/article/details/128144613

# DEBUG有用的命令
- tf树查询： `rosrun tf view_frames`
- 查询TF消息结构： `rosmsg show tf2_msgs/TFMessage`
```shell
geometry_msgs/TransformStamped[] transforms
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string child_frame_id
  geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion rotation
      float64 x
      float64 y
      float64 z
      float64 w
```
- 查询JointState消息结构： `rosmsg show sensor_msgs/JointState`
```shell
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string[] name
float64[] position  # 关节位置，单位：rad 或 m
float64[] velocity  # 关节速度，单位：rad/m 或 m/s
float64[] effort  # 关节受到的力，单位：N·m 或 N
```
- 查询关节关系图：`rosrun rqt_graph rqt_graph`
- 查询机械臂TF树：`rosrun rqt_tf_tree rqt_tf_tree`
- 报错`Waiting for frrobot/position_trajectory_controller/follow_joint_trajectory`
    - 解决办法：
- catkin_make -DCATKIN_WHITELIST_PACKAGES="fr5_real_moveit"

# 报错处理


# ROS兼容Python3
- 安装conda并激活环境
- 安装ros的包：
```shell
$ conda install catkin-tools rospkg
```
- 在python3的脚本中，加入python3的指定：
```python
#env/usr/bin python3
```
- chmod +x your/python_script.py
- rosruan package_name python_script

- https://blog.51cto.com/u_16099323/8237733