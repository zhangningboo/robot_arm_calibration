1.键盘控制
//需要先开启初始化节点（仅在单独开启键盘控制时需要开启 运行功能时已包括初始化节点 不需要重复开启）
# 运行时如果报错提示找不到master，注意是否已经修改小车中的.bashrc文件，将ROS_MASTER_URI和ROS_HOSTNAME全部改为小车的static ip
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
//开启键盘控制节点
roslaunch wheeltec_robot_rc keyboard_teleop.launch

2.巡线(雷达避障)
roslaunch simple_follower line_follower.launch 

3.雷达跟随。
roslaunch simple_follower laser_follower.launch 

4.视觉跟踪。
roslaunch simple_follower visual_follower.launch 

################################################################################   attention   ################################################################################
1.雷达的0度方向为其对应的tf指向的反方向,现在雷达的tf x轴指向小车的正中心,而激光雷达的0度指向为相反方向,两者相差了正好180度.
2.之前的在官方文件上作的标记,是雷达为顺时针为正,逆时针为负.现在被证实不成立,它和tf一样逆时针为正,顺时针为负.
3.之前的costmap存在虚空障碍物跳出问题,现在被证实是因为装配误差和可能是lsm10.cpp计算方式有点小bug,导致在屏蔽角230度的位置会有扫描到的小车框架的点作为跳出的障碍物出现在小车的微微斜后方
(在小车框架外,并不在小车本身,在框架外的原因可能和tof的计算点的方式有关).
4.可以确定的是屏蔽角最大最小值确实有正常工作(确实是进行角度屏蔽,屏蔽的位置也对,要注意一切在激光雷达的0度指向为其tf指向相反方向下进行分析).经过试验将最大屏蔽角改为232度可以有效防止虚空障碍物跳出.
################################################################################   attention   ################################################################################
#####################################################################################################
视觉和lidar联合建图:视觉生成的点云图和lidar生成的2D图可能对不上，视觉可用在初始重定位上，但受光照影响大。
激光重定位的问题是数据少，lidar重定位在对称型小的地图可用。
一般工程化时将充电桩作为重定位的起点，每次自动去找桩，这样就节省了在启动时的重定位算法。
导航工程化时一般将全局路径固定下来，在运行时进行替换。一般将目标容忍度设置的宽松一些，这样在到达目标位置附近后接收到reached消息后，手动进行精细导航。
#####################################################################################################
#####################################################################################################
在导航时要将"Ultrasonic_Avoid"的value设为true否则小车的tf坐标与雷达数据会相反
 <include file="$(find turn_on_wheeltec_robot)/launch/wheeltec_lidar.launch" >
   <arg name="Ultrasonic_Avoid" value="true"/>
  </include>
#####################################################################################################
5.2D建图、2D导航。（地图保存在小车中）
roslaunch turn_on_wheeltec_robot mapping.launch 

# robot size config file : turn_on_wheeltec_robot/costmap_common_params/param_four_wheel_diff_bs/costmap_common_params.yaml	detail:	ROS开发教程-chapter4.4
# 导航前先确定地图文件是否正确（detail:	ROS开发教程-chapter4.1）
roslaunch turn_on_wheeltec_robot navigation.launch 

5.0TEB调参
先运行
roslaunch turn_on_wheeltec_robot navigation.launch 
再运行
rosrun rqt_reconfigure rqt_reconfigure

5.1一键保存地图(WHEELTEC.pgm、WHEELTEC.yaml)
roslaunch turn_on_wheeltec_robot map_saver.launch 
保存地图至想要的文件夹
(1)先进入想要保存的文件夹下
(2)将地图以y_practice_map进行保存，如果之前有同名文件则进行覆盖
rosrun map_server map_saver -f y_practice_map

6.3D建图、3D导航。
roslaunch turn_on_wheeltec_robot 3d_mapping.launch 
roslaunch turn_on_wheeltec_robot 3d_navigation.launch 

7.纯视觉建图导航
roslaunch turn_on_wheeltec_robot pure3d_mapping.launch
roslaunch turn_on_wheeltec_robot pure3d_navigation.launch

8.语音控制
//开启底层、导航、雷达扫描节点
roslaunch xf_mic_asr_offline base.launch
//开启麦克风阵列初始化节点
roslaunch xf_mic_asr_offline mic_init.launch

9.KCF跟随
roslaunch kcf_track kcf_tracker.launch

10.自主建图 详见自主建图功能教程
//启动rrt_slam.launch文件：
roslaunch turn_on_wheeltec_robot rrt_slam.launch
//打开rviz，点击左下方“add”→“by topic”增加配置插件：
clicked_point	显示随机数的范围点和起点
detected_points	检测到的边界点
frontiers		滤波器接收到的边界点，数据同上
centroids		滤波后的有效边界点
global_detector_shapes	全局树
local_detector_shapes	本地树
//用rviz的publish point工具，按顺时针或者逆时针设置4个生长树的边界点，以及一个生长树起点（起点尽量靠近机器人起点），设置完成后机器人便依据生长树去探索地图。

11.WHEELTEC APP图传、建图与导航
//用户手机连接小车wifi，打开APP即可使用。该APP可控制小车移动、保存地图、查看摄像头画面，详见WHEELTEC APP功能教程
//图传，需要手动打开RGB摄像头节点：roslaunch usb_cam usb_cam-test.launch
APP端可以实时观看到摄像头画面
//建图，需要手动打开建图节点：roslaunch turn_on_wheeltec_robot mapping.launch 
APP端可以查看建图效果并保存，同时可以控制小车移动
//导航，需要手动打开导航节点：roslaunch turn_on_wheeltec_robot navigation.launch 
APP端可以控制小车移动

12.多机编队
//首先所有小车必须在同一个网络(wifi)下，然后修改.bashrc文件设置主从机，详见多机编队教程
//使用ssh命令远程登录之后，在分割终端中的左上角点击[广播到所有]，使用命令进行时间同步
sudo date -s "2021-01-30 08:48:00"
//主机开启导航节点
roslaunch turn_on_wheeltec_robot navigation.launch 
//从机1开启从动初始化节点
roslaunch wheeltec_multi wheeltec_slave.launch
//从机2开启从动初始化节点 (2个从机需要使用高性能路由器提供wifi)
roslaunch wheeltec_multi wheeltec_slave.launch
//主机开启控制从机节点
roslaunch wheeltec_multi robot_tf.launch

13.WEB浏览器显示摄像头
主机：roslaunch usb_cam usb_cam-test.launch
          rosrun web_video_server web_video_server
主机网页查看：http://localhost:8080/ (发出热点的为主机)
客户机网页查看：http://192.168.0.100:8080 (连接热点的为客户机)
【注】建议使用谷歌浏览器，经测试360极速浏览器、IE浏览器无法打开图像

14.物体识别(不支持树莓派4B 2GB)
物体识别：roslaunch ros_detection ros_tensorflow_classify.launch
查看摄像头实时画面：rqt_image_view(选择"/camera/rgb/image_raw/compressed"话题)

15.AR标签识别
roslaunch turn_on_wheeltec_robot ar_label.launch
创建一个二维码，边长为5，内容为0
rosrun ar_track_alvar createMarker -s 5 0
AR标签跟随
roslaunch simple_follower ar_follower.launch
   
16.2.4G无线手柄控制ROS端
//需要先开启初始化节点
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
//开启无线手柄控制节点
roslaunch wheeltec_joy joy_control.launch

17.深度学习
//开启深度学习节点
roslaunch darknet_ros darknet_ros.launch
//开启手势识别动作节点
roslaunch wheeltec_yolo_action gesture.launch
//开启沙盘运动节点
roslaunch wheeltec_yolo_action dp_drive.launch

18.Gazebo建图导航仿真
//2D gazebo建图
roslaunch wheeltec_gazebo_function mapping.launch
//键盘控制
roslaunch wheeltec_gazebo_function keyboard_teleop.launch
//保存地图
一键保存地图(WHEELTEC.pgm、WHEELTEC.yaml)
roslaunch wheeltec_gazebo_function map_saver.launch
//2D gazebo导航
roslaunch wheeltec_gazebo_function navigation.launch

19.自动回冲功能
1。使用建图功能建立机器人需要执行自动回充区域的地图。
roslaunch turn_on_wheeltec_robot mapping.launch
2。控制小车运动， 地图建立完毕后保存。
roslaunch turn_on_wheeltec_robot map_saver.launch
3。保存地图后， 关闭所有节点， 开启导航节点、 开启自动回充功能
roslaunch turn_on_wheeltec_robot navigation.launch
roslaunch auto_recharge_ros auto_recharger_node.launch
4。打开 rviz， 使用 2D Nav Goal 功能标记充电桩所在的位置
	1)打开 rviz 后， 右击 2D Nav Goal； 勾选 Tool Properties 选项
	2) 在弹出的窗口中， 找到 2D Nav Goal 话题， 修改为 “charger_position_update”
	3) 2D Nav Goal在地图中标注充电桩的实际位置和方向。
5。在自动回充终端按下键盘上的 q， 终端提示开始寻找充电桩。
6。停止充电， 在终端按下键盘上的 e 键即可。


------------------------------------------
其它常用命令

发布话题控制小车移动（发布速度）：
# pub 是发布，-r 是循环发布，10 是每秒10次（也就是10hz）。在输入pub后如果想使用TAB键自动补全功能，请不要先将-r和10加入，在已经补全后将其加入命令行。
# 而且在输入完 rostopic pub /cmd_vel geometry_msgs/Twist  后还要继续按TAB键进行补全。
# (官方2021.11.25更新，因为删除了速度平滑功能，所以手动发布速度命令后，如果希望小车停止运动，需要再发送一次速度为0的速度命令)
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist

发布话题控制小车移动（发布位置，代替rviz）：
# 后按TAB键进行补全。
# 其中frame_id 要填map。
# 我们设置的位置是相对于起始位置的位置。（在position中，只用到了x，y）
# orientation中只用到了z,w，朝向角度是逆时针为正，顺时针为负。朝向公式：z = sin(theta/2)，w = cos(theta/2)。theta为要朝向的角度。
# 相关视频为1.自主导航_直接上手使用，第15分开始。
rostopic pub /move_base_simple/goal geometry_msgs/PoseStampted 

递归修改当前(终端)文件夹下文件修改时间：
find ./* -exec touch {} \;

在工作空间下运行，安装ROS功能包全部依赖：（有时候提示缺什么可使用命令: sudo apt-get install ros-melodic-对应库）
rosdep install --from-paths src --ignore-src -r -y
有时候下面的更好用
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y

# 指定功能包编译：在编译单个功能包后运行 catkin_make 时编译的也是这个单个功能包。但重启terminal后catkin_make可能是对所有功能包进行操作，这个暂时没试出来。
# 我们在编译ros导航功能包时，可以发现在相关的工作空间下只有一个ROS_navigation的文件夹，而相关的细化的功能包（例如global_planner或local_planner。。）在这个文件夹下。
# 我们可以对其下的单个功能包进行单独编译而不用编译整个大包（整个大包进行编译时也是一个一个编译内部的小包）。
catkin_make -DCATKIN_WHITELIST_PACKAGES="功能包名"
# 解除指定功能包编译：此时我们编译的就是所有的功能包
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

使用豆瓣源进行pip安装(网速会快很多)：
pip install -i https://pypi.doubanio.com/simple/ python包名

# ssh登录：
# 添加选项“-Y”表示启用"X11 forwarding"功能。这个功能允许远程计算机上的应用程序将其图形界面显示在本地计算机上，使得用户可以在本地计算机上直接操作远程计算机上运行的图形化应用程序。
ssh -Y wheeltec@192.168.0.100

nfs挂载:
sudo mount -t nfs 192.168.0.100:/home/wheeltec/wheeltec_robot /mnt
nfs解除挂载:
sudo umount -t nfs 192.168.0.100:/home/wheeltec/wheeltec_robot /mnt

打开地图路径：
cd /home/wheeltec/wheeltec_robot/src/turn_on_wheeltec_robot/map
手动保存地图：
rosrun map_server map_saver -f 20200714

打开RGB摄像头
roslaunch usb_cam usb_cam-test.launch
rqt_image_view

打开深度摄像头
roslaunch astra_camera astrapro.launch 
rqt_image_view

查看节点与话题关系
rqt_graph
####################################################################
在tf树中odom到base_footprint的坐标转换为ekf提供，ekf的主要作用是进行odom坐标和gyro坐标合并后的滤波估值。如果只有odom或者gyro一个则odom到base_footprint坐标变换由robot提供，不需要ekf。
####################################################################
查看TF树
rosrun rqt_tf_tree rqt_tf_tree
查看话题数据图表
rqt_plot

查看话题列表
rostopic list
查看话题频率
rostopic hz /imu
查看单个话题属性信息
rostopic info /odom
查看单个话题实时信息（或打印该话题实时信息）
rostopic echo /odom

生成TF树pdf
rosrun tf view_frames

# 查看消息列表
rosmsg list
# 查看某型消息树,例：查看里程计消息树
rosmsg show nav_msgs/Odometry

# 查看节点列表
rosnode list
# 查看节点信息
rosnode info /wheeltec_robot

# 将新下载的包加载到 ros 的包的地址集合, 再执行roslaunch 时就不会再报错了
rospack profile

# bag 文件
第一个终端执行 roscore
第二个终端执行 rosbag info rslidar-outdoor-gps.bag 了解 bag 中 topic 的名称与类型
第三个执行 rosbag play rslidar-outdoor-gps.bag

# print string:
ROS_INFO_STREAM("Data ready"); 
ROS_ERROR_STREAM("wheeltec_robot can not open serial port,Please check the serial port cable! ");

# 在主机和笔记本同时连接同一个WIFI时除了要注意笔记本上的~/.bashrc文件的修改（200是主机的静态ip，202是笔记本的静态ip），还要注意修改主机上的~/.bashrc文件
################	笔记本上的~/.bashrc文件的修改	################
export ROS_MASTER_URI=http://192.168.3.200:11311
export ROS_HOSTNAME=192.168.3.202
################	笔记本上的~/.bashrc文件的修改	################

################### 主机上的~/.bashrc文件的修改 #########################
export ROS_MASTER_URI=http://192.168.3.200:11311
export ROS_HOSTNAME=192.168.3.200
################### 主机上的~/.bashrc文件的修改 #########################

#安装rtabmap功能包
sudo apt-get update (更新软件列表)
sudo apt-get install ros-melodic-rtabmap-ros（安装rtabmap）




