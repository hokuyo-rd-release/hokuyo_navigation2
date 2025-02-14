# EXPO_WizURG

## 全体構成
```
.
└── expo_wizurg/
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

ypspurのインストール
```
cd ~/catkin_ws/src
git clone https://github.com/BND-tc/yp-spur.git
cd yp-spur
mkdir build
cd build
cmake ..
make
sudo make install
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

cd ~/catkin_ws/src
git clone --recursive https://github.com/Hokuyo-RD/expo_wizurg.git
git clone https://github.com/Hokuyo-aut/ylm_ros
```
nmea_navsat_driver のインストール
```
sudo apt-get install ros-noetic-nmea-navsat-driver
```
rosdep による WizURGの依存関係パッケージのインストール
```
sudo apt-get install python3-rosdep

cd ~/catkin_ws/src
rosdep install -i --from-paths expo_wizurg
rosdep update
catkin_make or catkin build
```
.py .sh に実行権限を付与
```
cd ~/catkin_ws/src/expo_wizurg/src
chmod +x wizurg_navigation.py
chmod +x wizurg_waypoint_editor.py
chmod +x wizurg_waypoint_maker.py

cd ~/catkin_ws/src/expo_wizurg/scripts
chmod +x rosbag_mapping.sh
chmod +x waypoint_editor.sh
chmod +x waypoint_maker.sh
chmod +x wizurg_start.sh
```

## モータドライバインストールの確認
```
端末 1
ypspur-coordinator -d /dev/ttyUSB0 --blvr -p ~/catkin_ws/src/expo_wizurg/params/icart_ypspur_params/iCart3_100W.param

端末 2
cd ~/catkin_ws/src/yp-spur/build/sample
./run-test
```


## プログラムの実行
```
rosrun expo_wizurg wizurg_start.sh -オプション
```
シェルスクリプトの実行によりrosbag取得・マッピング・ナビゲーション・ウェイポイント作成を行う。`use_mapping`をtrueとした場合,ターミナル上で`save`と打つとマップが保存される。

### オプション一覧 (全6種類)
```
-B # 手動操作＋センサのrosbagを取得する。
-M # マッピングを実行する(オフライン)
-N # ナビゲーション(単一マップ)
-P # ナビゲーション(複数マップ)
-W # waypointの新規作成 
-E # waypointの編集
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-L # LIOモード: 各オプションの後につける。
```

### オプションファイルの構成
```
├── params
│   ├── wizurg_opt
│   │   └── 99_control_opt_lio.csv
│   │   └── 99_control_opt.csv
│   │   └── 99_edit_opt_lio.csv
│   │   └── 99_edit_opt.csv
│   │   └── 99_map_opt_lio.csv
│   │   └── 99_map_opt.csv
│   │   └── 99_plural_opt_lio.csv
│   │   └── 99_plural_opt.csv
│   │   └── 99_sensor_rosbag_lio.csv
│   │   └── 99_sensor_rosbag.csv
│   │   └── 99_way_opt_lio.csv
│   │   └── 99_way_opt.csv
│   └── maps_and_waypoints.csv
└── ...
```

### オプションファイル内のオプション一覧

| オプション | 指定値 | デフォルト値 | 意味 | 自由欄 |
| ---- | ---- | ---- | ---- | ---- |
|use_joy_controller|true|true|手動操作
|use_mapping|false|true|Gmappingの起動 (単一のマップと単一のwaypointを作成)
|use_navigation|true|false|ナビゲーションの起動
|use_sensors|true|true|北陽LiDAR、カメラ、Ichimilの起動
|use_icart_driver|true|true|icart_driver_tf の起動
|use_lio|true|false|Hokuyo_LIOの起動
|use_unity_sim|false|false|Unity シミュレータで使う際の
|use_ypspur|true|true|ypspurモータドライバの起動
|use_multi_maps|true|false|ナビゲーション時の複数マップの読み込み(読み込みファイルはmaps_and_waypoints.csvに記載する。)
|map_file_name|LIO_sim_test|test_map|Gmappingの保存ファイル名|
|waypoint_file_name|LIO_sim_test|test_waypoints|waypointの保存ファイル名
|rosbag_record|true|false|rosbag の記録
|rosbag_dir|/media/kev/samsung/ATC2024/raw_rosbag/nav_bag/atc|~/rosbag|rosbagファイルの保存先
|make_waypoints|false|false|waypoint_makerの起動|
|edit_waypoints|false|false|waypoint_editorの起動|

### ①ROSBAGセンサデータ取得
```
rosrun wizurg wizurg_start.sh -B　-L
```

### ②地図作成
この際、raw_rosbag には、後処理したものは使わないようにする。(tf_remover等。)
wheelオドメトリが入っているものでも良い。

#### 2-1 p2o用にトピックを抽出した別のrosbag ファイルを作成する
~/github/hokuyo_slam下のrosbagディレクトリに、

```
cd ~/github/hokuyo_slam
./get_rosbag.bash <raw_rosbag> <lio, pc, fix topic rosbag>
# ex. ./get_rosbag.bash toyonaka.bag toyonaka_test_input.bag
```

#### 2-2 2-1で作成したrosbag を使って絶対座標の情報を付与した3D地図を作成
この際、システム用に相対座標に変換した3D点群地図も作成
```
cd ~/github/hokuyo_slam
./hokuyo_slam.bash <lio, pc, fix topic rosbag> <mapname>
# ex. ./hokuyo_slam.bash toyonaka_test_input.bag toyonaka_test
```
#### 2-3 2D地図作成
pcd_to_pgm を用いて、3Dの相対座標の点群地図を圧縮し、2Dの点群地図を作成する。
```
rosrun expo_wizurg wizurg_start.sh -M -L
```
#### 2-4 2D地図修正
地図の障害物と通行可能領域の整合性を保つため、
適宜、GIMPを用いて2D地図を手作業で修正する。(3D点群と2D点群を重ね合わせたり、Waypoint を参考にする。)

### ③ Waypoint の新規作成
terminal 2のrosbag は 車輪オドメトリのtf が無いものを使用し、--clock オプションを使うこと。
tfを消したもの、最初からrecordしていないもののどちらでも構わない。
```
# terminal 1
rosrun wizurg wizurg_start.sh -W -L

# terminal 2
rosbag play <mapnamebag> --clock
```
### ④ Waypoint の編集
```
rosrun wizurg wizurg_start.sh -E -L
```

### ⑤ 複数マップ・Waypointでのナビゲーション
`./expo_wizurg/config/maps_and_waypoints.csv`に、複数地図名とそれに対応するウェイポイントを記入する.
```
rosrun wizurg wizurg_start.sh -P -L
```

### hokuyo-lioありのrosbagから点群地図作成
```
roslaunch test_tools hlio_make_pcd.launch 

別端末でrosbagを再生する.

```
wizurg_ros1/map/map.pcd　が生成される.


### LIOを使用する場合
上記各オプションに`-L`を追加する.  
例：ナビゲーションの場合）
```
rosrun wizurg wizurg_start.sh -P -L
```

## コントローラー設定
前進  :   十字ボタン上  
後進  :   十字ボタン下  
左回転:    Yボタン  
右回転:    Aボタン  
通常モード:Lボタン押しながら操作  
高速モード:Rボタン押しながら操作  
低速モード：（通常モードボタン ＋ 高速モードボタン）を押しながら操作  