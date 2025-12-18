# hokuyo_navigation2

`hokuyo_navigation2`は、北陽電機製の3D LiDAR（RSFセンサ）を使用したROS 2ベースのナビゲーションシステムです。
3D-SLAMによる自己位置推定とROS 2 Navigation Stack (Nav2)を連携させ、高精度な2D自律移動を実現します。

また、直感的な操作を可能にするWebベースのGUI `hokuyo_navigation2_gui` を同梱しており、マッピングからナビゲーションまでの一連の操作をブラウザから簡単に行うことができます。

!画像

## 主な機能

- **3D SLAMと2Dウェイポイントファイル出力の同時実行**:
  - `hokuyo_lio` を用いた高精度なLiDAR慣性オドメトリ（LIO）と3D点群マップ生成。
  - ROS Bagから`lio_raw`（軌跡ベース）または`p2o`（点群ベース）の3Dマップ（`.pcd`）を作成。
  - 3Dマップ生成と同時にウェイポイント作成
  - 3DマップからNav2用の2Dグリッドマップ（`.pgm`, `.yaml`）へ変換。

- **2Dナビゲーション**:
  - 3D SLAMの出力（`simple_fastlio_localization`）を利用したリアルタイム自己位置推定。
  - Nav2 (Navigation2) スタックと連携し、指定されたウェイポイントに沿った自律走行。
  - 単一マップ走行および複数マップを連続して走行するマルチマップナビゲーションに対応。

- **Webベースの統合GUI (`hokuyo_navigation2_gui`)**:
  - **プロセス実行**: データ取得、マッピング、ナビゲーションの各プロセスをブラウザから起動。
  - **ファイル管理**: マップ、ウェイポイント、設定ファイルなどをブラウザ上で管理（作成、名前変更、削除）。
  - **高機能エディタ**:
    - **Map Viewer**: 3D/2Dマップとウェイポイントを視覚化し、GUI上で直感的にウェイポイントを編集（追加、移動、回転、属性変更）。
    - **CSV Editor**: マルチマップ走行シナリオをテーブル形式で簡単に編集。

- **Docker対応**:
  - 依存関係を含んだ開発・実行環境をDockerコンテナとして提供し、セットアップを簡素化。

## 依存関係

### システムツール
以下のツールがシステムにインストールされている必要があります。
```bash
sudo apt-get update
sudo apt-get install -y tree xdotool wmctrl zenity
```

### ROS 2 パッケージ
本パッケージは以下のROS 2パッケージに依存しています。ワークスペースにクローンしてビルドしてください。
- icart_mini_driver_ros2: iCart-miniのROS 2ドライバ（ロボットベースとして使用する場合）。
- hokuyo_slam_ros2: `p2o`マッピングで使用。
- その他、`simple_fastlio_localization`や`fix2xyz`など、プロジェクトで利用される各種パッケージ。

### Python パッケージ
Web GUI (`hokuyo_navigation2_gui`) を使用するために必要なPythonパッケージです。
```bash
pip3 install flask flask-sockets gevent gevent-websocket websockets pyyaml
```

## ビルド

1.  **ワークスペースのセットアップ**:
    ROS 2ワークスペースを作成し、`src`ディレクトリに本パッケージと依存パッケージをクローンします。

2.  **実行権限の付与**:
    スクリプトに実行権限を付与します。
    ```bash
    cd <your_colcon_ws>/src/hokuyo_navigation2/hokuyo_navigation2
    chmod +x scripts/*.sh scripts/*/*.sh src/*.py
    ```

3.  **ビルド**:
    ワークスペースのルートで`colcon build`を実行します。
    ```bash
    cd <your_colcon_ws>
    colcon build
    colcon build --symlink-install --packages-select simple_fastlio_localization lio_nav2_bringup hokuyo_navigation2
    ```

## 実行方法

本システムは、Dockerを使用する方法と、ホストマシンで直接実行する方法があります。
GUIからの操作が推奨されますが、CUIから`coordinator.sh`スクリプトを実行することも可能です。

### 方法1: DockerとWeb GUIを使用する (推奨)

1.  **Dockerコンテナの起動**:
    プロジェクトルートにある`docker/run.bash`スクリプトでコンテナをビルド・起動します。
    ```bash
    # 例: コンテナ名を "hokuyo_navigation2_dev" に設定
    ./docker/run.bash -n hokuyo_navigation2_dev
    ```

2.  **サーバーの起動**:
    コンテナ内で、`hokuyo_navigation2_docker_server.bash` を実行して、WebサーバーとVizantiサーバーを起動します。
    ```bash
    # コンテナに接続
    docker exec -it hokuyo_navigation2_dev /bin/bash
    
    # サーバー起動スクリプトを実行
    ./scripts/00_sample_util/hokuyo_navigation2_docker_server.bash
    ```

3.  **GUIへのアクセス**:
    Webブラウザで `http://<DockerホストのIPアドレス>:5050` にアクセスします。
    GUIの指示に従い、マッピングやナビゲーションを実行してください。
    詳細は `hokuyo_navigation2_gui` のドキュメントを参照してください。

### 方法2: CUIから実行する

`coordinator.sh`は、Zenityを利用したメニューを通じて、マッピングやナビゲーションなどの各機能を実行するための統合スクリプトです。

1.  **環境設定**:
    ROS 2環境をセットアップします。
    ```bash
    source /opt/ros/humble/setup.bash
    cd <your_colcon_ws>
    source install/setup.bash
    ```

2.  **コーディネータの実行**:
    スクリプトを実行すると、実行したい機能を選択するダイアログが表示されます。
    ```bash
    ros2 run hokuyo_navigation2 coordinator.sh
    ```
     <!-- Zenityメニューのスクリーンショットを挿入 -->

    **選択可能なオプションの例**:
    - `get_rosbag`: データ取得用のROS Bagを生成。
    - `hokuyo_slam`: `p2o`アルゴリズムでマッピングを実行。
    - `map_opt`: PCDマップをPGMに変換。
    - `way_opt`: ウェイポイントを作成。
    - `nav_opt`: 単一マップでナビゲーションを実行。
    - `plural_opt`: 複数マップでナビゲーションを実行。
    - `pcd_opt`: `lio_raw`でマッピングを実行。

    選択後、ファイル選択ダイアログなどが表示されるので、指示に従って操作してください。

## パッケージ構成

- `/config`: Nav2、`hokuyo_lio`、`coordinator.sh`などの設定ファイル。
- `/data`: `init_pose.txt`など、マップごとの初期位置情報を格納。
- `/launch`: 各種機能（ナビゲーション、マッピング）を起動するためのROS 2 Launchファイル。
- `/map`: 生成されたマップファイル（`.pcd`, `.pgm`, `.yaml`）のデフォルト保存場所。
- `/scripts`: `coordinator.sh`やマッピング処理など、主要な処理を実行するシェルスクリプト群。
- `/src`: C++やPythonで実装されたカスタムROS 2ノード（例: `pcd_tf_extractor.py`）。
- `/urdf`: ロボットモデルのURDFファイル。
- `/waypoints`: 作成されたウェイポイントファイル（`.json`）のデフォルト保存場所。
- `/hokuyo_navigation2_gui`: Web GUIのソースコード。