# LIO_Localization のサンプル実行手順

## ビルド

icart3のインストール
```
cd ~/catkin_ws/src
git clone https://github.com/Hokuyo-RD/icart
```
hokuyo パッケージ群 (urg_node, hokuyo3d, base_local_planner, ylm_ros, expo_wizurg)
```
sudo apt-get install ros-noetic-urg-node
sudo apt-get install ros-noetic-hokuyo3d
sudo apt-get install ros-noetic-ira-laser-tools
sudo apt-get install ros-noetic-gmapping
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-dwa-local-planner
sudo apt-get install ros-noetic-base-local-planner
sudo apt-get install ros-noetic-jsk-rviz-plugins
sudo apt-get install ros-noetic-pointcloud-to-laserscan

cd ~/catkin_ws/src
git clone --recursive https://github.com/Hokuyo-RD/expo_wizurg.git
git clone https://github.com/Hokuyo-RD/ylm_ros
```
pointcloud_to_laserscan
```
git clone https://github.com/Hokuyo-RD/pointcloud_to_laserscan.git
```
nmea_navsat_driver のインストール
```
sudo apt-get install ros-noetic-nmea-navsat-driver
git clone https://github.com/Hokuyo-RD/nmea_navsat_driver.git
catkin build nmea_navsat_driver -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF
catkin build nmea_navsat_driver
```
rosdep による WizURGの依存関係パッケージのインストール
```
sudo apt-get install python3-rosdep

cd ~/catkin_ws/src
rosdep install -i --from-paths expo_wizurg
rosdep update
catkin_make or catkin build
```

## ファイルを以下の構成とする
```
.
└── expo_wizurg/
    ├── rosbag/
    │   └── toyonaka_0328.bag
    ├── map/    
    │   └── toyonaka_0328_test.pcd
    ...
```

## プログラムの実行手順
```
# 端末 1
roscd expo_wizurg
roslaunch expo_wizurg lio_localization.launch

# 端末 2
rosbag play toyonaka_0328.bag

```