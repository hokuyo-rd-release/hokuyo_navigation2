# hokuyo_navigation2

`hokuyo_navigation2`は、北陽電機製の3D LiDAR（RSFセンサ）専用のROS 2ベースのナビゲーションシステムです。
3D-SLAMによる自己位置推定とROS 2 Navigation Stack (Nav2)を連携させ、高精度な2D自律移動を実現します。

また、直感的な操作を可能にするWebベースのGUI `hokuyo_navigation2_gui` を用いることで、マッピングからナビゲーションまでの一連の操作をブラウザから簡単に行うことができます。

![hokuyo_navigation2](Image/hokuyo_navigation2.png)

## 主な機能

- **3D SLAMと2Dウェイポイントファイル出力の同時実行**:
  - `hokuyo_lio` を用いた高精度なLiDAR慣性オドメトリ（LIO）と3D点群マップ生成。
  - ROS Bagから`lio_raw`（軌跡ベース）または`p2o`（点群ベース）の3Dマップ（`.pcd`）を作成。
  - 3Dマップ生成と同時にウェイポイント作成
  - 3DマップからNav2用の2Dグリッドマップ（`.pgm`, `.yaml`）へ変換。

- **2Dナビゲーション**:
  - Hokuyo RSF センサのGNSSとHokuyo LIO (LiDAR Inertial Odometry)出力によるリアルタイム自己位置推定を用いた自律走行
  - 3D点群マップと`simple_fastlio_localization` を利用したリアルタイム自己位置推定を用いた自律走行
  - Nav2 (Navigation2) スタックと連携し、指定されたウェイポイントに沿った自律走行。
  - 単一マップ走行および複数マップを連続して走行するマルチマップナビゲーションに対応。

- **Webベースの統合GUI (`hokuyo_navigation2_gui`)**:
  - **プロセス実行**: データ取得、マッピング、ナビゲーションの各プロセスをブラウザから起動。
  - **ファイル管理**: マップ、ウェイポイント、設定ファイルなどをブラウザ上で管理（作成、名前変更、削除）。
  - **高機能エディタ**:
    - **Map Viewer**: 3D/2Dマップとウェイポイントを視覚化し、GUI上で直感的にウェイポイントを編集（追加、移動、回転、属性変更）。
    - **CSV Editor**: マルチマップ走行シナリオをテーブル形式で簡単に編集。

## 依存関係

### システムツール
以下のツールがシステムにインストールされている必要があります。
```bash
sudo apt-get update
sudo apt-get install -y tree xdotool wmctrl zenity
```

### ROS 2 パッケージ
本パッケージは以下のROS 2パッケージに依存しています。
詳細は`hokuyo_navigation2` でまとめてクローン・ビルドする方法が記載されているので参考にしてください。

- **HOKUYO_RSF**
  - Hokuyo RSF センサの ROS2 パッケージです。GNSSとLiDAR Inertial Odometry (LIO) の相互変換による位置出力 (Odometry, fix) を提供します。
- **vizanti**:
  - Webブラウザ上でROSトピックを可視化するためのツール。
  - `hokuyo_navigation2_gui` の Map Viewer 機能のバックエンドとして使用されます。
- **rosbridge_suite**
  - websocket を使ってウェブで ROS Topic 通信を実現するパッケージ vizanti が依存
- **jsk_visualization**
  - RViz2 のカスタムヴィジュアルプラグイン
- **icart_mini_driver_ros2**:
  - T-frog project 製のロボットベース `iCart-mini` 用のROS 2ドライバ。ロボットを制御する場合に必要です。
- [**hokuyo_slam_ros2**](https://github.com/hokuyo-rd/hokuyo_slam_ros2.git):
  - 3D SLAM アルゴリズム `p2o` を提供するパッケージ。
  - 高精度な3D点群マップの生成に使用されます。
- **simple_fastlio_localization**:
  - LIOベースの自己位置推定パッケージ。
  - 事前に作成した3Dマップ上での現在のロボット位置を推定します。
- **fix2xyz_packages**:
  - GNSSデータ (NavSatFix) を直交座標系 (XYZ) に変換するツール。
  - GNSSを使用したナビゲーションやマッピングで使用されます。
- **lio_nav2_bringup**:
  - LIOとNav2を連携させて起動するためのLaunchファイル群を含むパッケージ。
- **waypoint_manager**
  - Nav2 に ウェイポイントを送信するノード

### Python パッケージ
```bash
pip3 install -r src/requirements.txt
```

```bash
pip3 install pipreqs # pipreqs で .py ファイルの 依存パッケージをrequirements.txt に格納。
pipreqs src/ # requirements.txt を生成
```


## ビルド

1.  **ワークスペースのセットアップ**:
    ROS 2ワークスペースを作成し、`src`ディレクトリに本パッケージをクローンしrosdep コマンドで依存関連パッケージをインストールします。

    ```bash
    cd <your_colcon_ws>
    git clone https://github.com/hokuyo-rd/hokuyo_navigation2.git
    rosdep install -i --from-path src/ -y
    ```

1.  **実行権限の付与**:
    スクリプトに実行権限を付与します。
    ```bash
    cd <your_colcon_ws>/src/hokuyo_navigation2/hokuyo_navigation2
    chmod +x scripts/*.sh scripts/*/*.sh src/*.py
    ```

2.  **ビルド**:
    ワークスペースのルートで`colcon build`を実行します。
    ```bash
    cd <your_colcon_ws>
    colcon build
    colcon build --symlink-install --packages-select hokuyo_navigation2
    ```

## 実行方法

本システムは、GUIからの操作とCUIから`coordinator.sh`スクリプトを直接実行する方法があります。GUIからの操作が推奨されますが、CUIから`coordinator.sh`スクリプトを実行することも可能です。


### 方法1: Web GUIを使用する

コンテナ内で、`hokuyo_navigation2_docker_server.bash` を実行して、WebサーバーとVizantiサーバーを起動します。
```bash
# サーバー起動スクリプト
./scripts/00_sample_util/start_server.bash
```
```bash
# サーバー停止スクリプト
./scripts/00_sample_util/stop_server.bash
```

Webブラウザで `http://<ホストマシンのIPアドレス>:5050` にアクセスします。GUIの指示に従い、マッピングやナビゲーションを実行してください。詳細は `hokuyo_navigation2_gui` のドキュメントを参照してください。

### 方法2: CUIから実行する

`coordinator.sh`は、Zenityを利用したメニューを通じて、マッピングやナビゲーションなどの各機能を実行するための統合スクリプトです。

**coordinatorの実行**:
    スクリプトを実行すると、実行したい機能を選択するダイアログが表示されます。
```bash
ros2 run hokuyo_navigation2 coordinator.sh
```
![coordinator](Image/coordinator.png)

**選択可能なオプションの例**:
- `start_get_rosbag`: ROS Bag を取得するためにセンサやロボットのノードを起動します。
- `start_mapping`: 3D SLAMと2Dウェイポイントファイル出力の同時実行: ROS Bagファイルを使用して、以下のSLAMアルゴリズムを実行します。SLAM実行時にウェイポイントファイル.jsonが作成されます。
    - p2o (hokuyo_slam を用いた3D点群マップ) 
    - lio_raw (LiDAR Inertial Odometry の軌跡に基づく3D点群マップ)
    - PCDからPGMへの変換: 3D点群マップ`.pcd`を2D占有格子マップ`.pgm`と`.yaml`に変換します。オプションとして、変換の際に.json 形式のウェイポイントファイルを指定すると、`.pgm`マップ状に、指定したウェイポイントに沿って通行可能領域を生成します。
- `start_navigation`: 
    - 単一マップ走行: 指定したマップとウェイポイントファイルを使用して自律走行を開始します。自律走行は実行時に、GNSSベースか、LiDARベースの自律走行のどちらか1つを選択できます。
    - マルチマップ走行: 複数のマップとウェイポイントを順番に走行するシナリオをCSVファイルで定義し、実行します。

選択後、ファイル選択ダイアログなどが表示されるので、指示に従って操作してください。

## パッケージ構成

- `/config`: Nav2、`hokuyo_lio`、`coordinator.sh`などの設定ファイル。
- `/data`: `init_pose.txt`など、マップごとの初期位置情報を格納。
- `/launch`: 各種機能（ナビゲーション、マッピング）を起動するためのROS 2 Launchファイル。
- `/map`: 生成されたマップファイル（`.pcd`, `.pgm`, `.yaml`）のデフォルト保存場所。
- `/scripts`: `coordinator.sh`やマッピング処理など、主要な処理を実行するシェルスクリプト群。
- `/src`: C++やPythonで実装されたカスタムROS 2ノードと ROS Bag デシリアライズツール。
- `/urdf`: ロボットモデルのURDFファイル。
- `/waypoints`: 作成されたウェイポイントファイル（`.json`）のデフォルト保存場所。