# ndtg_mapping
ndt_mapping + gps(rtk)

# 依赖
## depend
rosdep update --include-eol-distros

export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml

rosdep check --from-path src

sudo apt install ros-noetic-autoware-config-msgs ros-noetic-autoware-msgs ros-noetic-jsk-rviz-plugins ros-noetic-velodyne-pointcloud

## gtsam
wget -O ~/tools/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip

cd ~/tools/ && unzip gtsam.zip -d ~/tools/

cd ~/tools/gtsam-4.0.0-alpha2/

mkdir build && cd build

cmake ..

sudo make install


# 编译
cd ~/catkin_ndtg/src

git clone git@github.com:yanjingang/ndtg_mapping.git
cd ..

catkin_make


# 测试
source devel/setup.bash

roslaunch ndtg_mapping run.launch

