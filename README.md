# hokuyo_navigation2

万博でのSPELデモンストレーションを目的としたナビゲーションソフトウェアのROS2版です。
3D自己位置推定の結果をNavigation Stack に渡して、2DのPath Planningにより
2Dの自律移動を実現します。[expo_wizurg](https://github.com/Hokuyo-RD/expo_wizurg)からのアップデート、システム構成の発案、パッケージの選定とインテグレーションを北陽電機 髙橋が作成/実施しました。

## 全体構成
```
.
└── hokuyo_navigation2/
    ├── src/
    │   ├── controller_setup.cpp
    │   ├── wizurg_navigation.py
    │   ├── wizurg_waypoint_editor.py
    │   └── wizurg_waypoint_maker.py
    ├── launch/
    │   ├── control_setup.launch
    │   ├── start.launch
    │   └── ...
    ├── scripts
    │   └── ...
    ├── config
    │   └── ...
    ├── map
    │   └── ...
    ├── waypoints
    │   └── ...
    └── urdf
         └── ...
```

## ビルド

linuxターミナルコマンドのインストール
```
sudo apt-get install tree
sudo apt-get install xdotool
sudo apt-get install wmctrl

```

UAMノード       : https://github.com/f-wada/safety_urg_node2  
SPELコア技術    : https://github.com/Hokuyo-RD/fusion_tools_ros2  
緯度経度-マップ座標変換: https://github.com/Hokuyo-RD/fix2xyz_packages_ros2  

ypspurのインストール
```
cd ~/colcon_ws/src
git clone https://github.com/BND-tc/yp-spur.git
cd yp-spur
mkdir build
cd build
cmake ..
make
sudo make install
```

icart3のインストール
```
cd ~/colcon_ws/src
git clone https://github.com/Hokuyo-RD/icart_mini_driver_ros2
```
hokuyo パッケージ群 (urg_node, hokuyo3d, base_local_planner, ylm_ros, hokuyo_navigation2)
```
cd ~/colcon_ws/src
git clone -b okamoto_devel --recursive https://github.com/Hokuyo-RD/hokuyo_navigation2.git
```
pointcloud_to_laserscan
```
git clone -b humble https://github.com/Hokuyo-RD/pointcloud_to_laserscan_ros2.git
```
nmea_navsat_driver のインストール
```
git clone -b ros2 https://github.com/Hokuyo-RD/nmea_navsat_driver_ros2.git
```
rosdep による WizURGの依存関係パッケージのインストール
```
sudo apt-get install python3-rosdep

cd ~/colcon_ws/src
rosdep install -i --from-paths hokuyo_navigation2
rosdep update
colcon build --symlink-install
```
.py .sh に実行権限を付与
```
cd ~/colcon_ws/src/hokuyo_navigation2/src
chmod +x wizurg_navigation.py
chmod +x wizurg_waypoint_editor.py
chmod +x wizurg_waypoint_maker.py

cd ~/colcon_ws/src/hokuyo_navigation2/scripts
chmod +x rosbag_mapping.sh
chmod +x waypoint_editor.sh
chmod +x waypoint_maker.sh
chmod +x wizurg_start.sh
```

## モータドライバインストールの確認
```
端末 1
ypspur-coordinator -d /dev/ttyUSB0 --blvr -p ~/colcon_ws/src/hokuyo_navigation2/params/icart_ypspur_params/iCart3_100W.param

端末 2
cd ~/colcon_ws/src/yp-spur/build/sample
./run-test
```
### Docker の場合 
colcon build --symlink-install --packages-select icart_mini_driver
でビルドしないとシェルスクリプトに実行権限が付与されない。

```
sudo chown -R root:root /home/ubuntu/colcon_ws
```
## プログラムの実行手順

### プログラムの実行の流れ
