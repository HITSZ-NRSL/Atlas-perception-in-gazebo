半实物仿真平台说明：

硬件配置，默认PC与Atlas系统为Ubuntu18.04, 预装ROS Melodic和Gazebo-9.0，在PC上运行Gazebo仿真环境，将无人机上接受的图片传到Atlas200DK上，通过CANN框架推理YOLOX感知算法，再将检测结果传回PC上。

依赖安装：
sudo apt install ninja-build exiftool ninja-build protobuf-compiler libeigen3-dev genromfs xmlstarlet libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python-pip python3-pip gawk

pip install --upgrade setuptools
python -m pip install --upgrade pip
pip2 install pandas jinja2 pyserial cerberus pyulog==0.7.0 numpy toml pyquaternion empy pyyaml 
pip3 install packaging numpy empy toml pyyaml jinja2 pyargparse

ROS安装：
根据观望wiki.ros.org/Installation/Ubuntu安装好ROS

Gazebo安装：
sudo apt-get remove gazebo* 
sudo apt-get remove libgazebo*
sudo apt-get remove ros-melodic-gazebo* #kinetic noetic对应修改
sudo apt-get install ros-melodic-moveit-msgs ros-melodic-object-recognition-msgs ros-melodic-octomap-msgs ros-melodic-camera-info-manager  ros-melodic-control-toolbox ros-melodic-polled-camera ros-melodic-controller-manager ros-melodic-transmission-interface ros-melodic-joint-limits-interface
sudo apt install gazebo9 libgazebo9-dev

mavros安装：
sudo apt install ros-kinetic-mavros ros-kinetic-mavros-extras 		# for ros-kinetic
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras 		# for ros-melodic
wget https://gitee.com/robin_shaun/XTDrone/raw/master/sitl_config/mavros/install_geographiclib_datasets.sh

sudo chmod a+x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh #这步需要装一段时间

px4安装：
git clone https://github.com/PX4/PX4-Autopilot.git
mv PX4-Autopilot PX4_Firmware
cd PX4_Firmware
git checkout -b xtdrone/dev v1.11.0-beta1
git submodule update --init --recursive
make px4_sitl_default gazebo

然后修改~/.bashrc
source ~/catkin_ws/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo

再测试一下px4
source ~/.bashrc
cd ~/PX4_Firmware
roslaunch px4 mavros_posix_sitl.launch

rostopic echo /mavros/state
如果上面的rostopic命令显示connected: True，说明MAVROS与gazebo通信成功

编译PC代码：
在src/同目录下
catkin build

多机通信设置：
PC端，在.bashrc中写入
export ROS_IP=192.168.1.166
export ROS_MASTER_URI=http://192.168.1.166:11311
Atlas端，在.bashrc中写入
export ROS_IP=192.168.1.2
export ROS_MASTER_URI=http://192.168.1.166:11311

PC半实物仿真环境开启：
cp scripts/launch/* ~/PX4_Firmware/launch/

打开一个终端,
source devel/setup.bash
roslaunch px4 iris_realsense_empty.launch

打开一个终端，
source devel/setup.bash
python scripts/communication/multirotor_communication.py iris 0

打开一个终端，
source devel/setup.bash
python scripts/control/control_uav_flying.py

此时可以看到无人机飞到桌子前

编译Atlas代码：
启动Atlas并与主机通过ssh连接后，将scripts/YOLO_ws文件夹拷到Atlas上，然后进行编译
cd YOLO_ws/
catkin build

Atlas感知算法开启：
source devel/setup.bash
cd src/yolov3_atlas_ros/yolo_atlas_ros/src
python3 acl_ros_yolox_raw.py
看到打印检测坐标时，表示感知成功
