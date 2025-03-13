# プログラムの実行手順

## プログラムの実行の流れ
```
rosrun expo_wizurg wizurg_start.sh

# 1. 指示されたキーとenterを入力

select operate_mode 

1) control_opt        ... manual operation only
2) sensor_rosbag      ... manual operation and rosbag record
3) map_opt            ... mapping 
4) way_opt            ... make waypoints 
5) edit_opt           ... edit waypoints
6) nav_opt            ... single_map navigation
7) plural_opt         ... multi_map navigation

# 2. 指示されたキーとenterを入力

select odom_type 

1) icart_mini_driver
2) LIO
```

## オプションファイルの構成
```
├── params
│   ├── wizurg_opt
│   │   └── 99_common_opt_lio.csv
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

# 0. センサから取得したデータに名前をつける。
99_common_opt_lio.csvのmap_file_name、waypoint_file_nameに、
データの名前をつける。

# 1. ROSBAGセンサデータ取得

```
rosrun expo_wizurg wizurg_start.sh 1) → 2)
```

# 2. 地図作成

## 2.1. hokuyo_lioのデータから点群地図作成
この際、raw_rosbag には、後処理したものは使わないようにする。(tf_remover等。)
wheelオドメトリが入っているものでも良い。

```
# terminal 1
roslaunch test_tools hlio_make_pcd.launch 
# terminal 2
rosbag play <mapname_bag>

# ディレクトリを作成：~/github/hokuyo_slam/data/$MAP_NAME 
# ディレクトリ下に、init_pose.txt を作成し、0.0,0.0,0.0,0.0,0.0,0.0,1.0 とする。(これがlocalizationの初期値となる。)
```
expo_wizurg/map/$MAP_NAME.pcd　が生成される.

## 2.2. p2oを使って点群地図作成
### 2.2.1. p2o用にトピックを抽出した別のrosbag ファイルを作成する

1. ROSBAGセンサデータで取得したrosbag にhokuyo_lioのトピックだけを抜いたものを用意する。
2. ~/github/hokuyo_slam下のrosbagディレクトリにで取得したrosbag を設置する。

```
cd ~/github/hokuyo_slam
./get_rosbag.bash <raw_rosbag> <lio, pc, fix topic rosbag>
# ex. ./get_rosbag.bash toyonaka.bag toyonaka_test_input.bag
```

### 2.2.2. 2.2.1. で作成したrosbag を使って絶対座標の情報を付与した3D地図を作成
この際、システム用に相対座標に変換した3D点群地図とGNSSで取得した初期値も設定される。
```
cd ~/github/hokuyo_slam
./hokuyo_slam.bash <lio, pc, fix topic rosbag> <mapname>
# ex. ./hokuyo_slam.bash toyonaka_test_input.bag toyonaka_test
```
## 2.3. 2D地図作成
```
rosrun expo_wizurg wizurg_start.sh 3) → 2)

# 2D地図がRVizに表示されたら"save"と入力して保存

save
```
pcd_to_pgm を用いて、3Dの相対座標の点群地図を圧縮し、2Dの点群地図を作成する。
シェルスクリプトの実行によりrosbag取得・マッピング・ナビゲーション・ウェイポイント作成を行う。`use_mapping`をtrueとした場合,ターミナル上で`save`と打つとマップが保存される。

## 2.4. 2D地図修正
地図の障害物と通行可能領域の整合性を保つため、
適宜、GIMPを用いて2D地図を手作業で修正する。(3D点群と2D点群を重ね合わせたり、Waypoint を参考にする。)

# 3. Waypoint の新規作成
terminal 1のrosbag は 車輪オドメトリのtf が無いものを使用し、--clock オプションを使うこと。
tfを消したもの、最初からrecordしていないもののどちらでも構わない。

```
# terminal 1
rosbag play <mapname_bag> --clock

# terminal 2
rosrun expo_wizurg wizurg_start.sh 4) → 2)
```

# 4. Waypoint の編集
```
rosrun expo_wizurg wizurg_start.sh 5) → 2)
```

# 5. 単一マップ・Waypointでのナビゲーション

```
rosrun expo_wizurg wizurg_start.sh 6) → 2)
```

# 6. 複数マップ・Waypointでのナビゲーション
`./expo_wizurg/config/maps_and_waypoints.csv`に、複数地図名とそれに対応するウェイポイントを記入する。
```
rosrun expo_wizurg wizurg_start.sh 7) → 2)
```

# コントローラー設定
前進  :   左スティック上  
後進  :   左スティック下  
左回転:    Yボタン  
右回転:    Aボタン  
通常モード:Lボタン押しながら操作  
高速モード:Rボタン押しながら操作  
低速モード：（通常モードボタン ＋ 高速モードボタン）を押しながら操作  