# hokuyo_navigation2

RSFセンサ専用のROS 2ナビゲーションソフトウェアです。3D自己位置推定の結果をROS 2 Navigation Stack (Nav2) と連携させ、2Dの自律移動を実現します。

## ビルド

linuxターミナルコマンドのインストール
```
sudo apt-get install tree
sudo apt-get install xdotool
sudo apt-get install wmctrl
```

一例としてロボットベースは、icart_mini を用いています。本パッケージで使用する際に、icart_mini の ROS 2 ドライバ [icart_mini_driver_ros2](https://github.com/hokuyo-rd/icart_mini_driver_ros2.git) を用いています。


.py .sh に実行権限を付与
```
sudo chmod +x scripts/*.sh　src/*.py
```

### Docker の場合 
colcon build --symlink-install --packages-select icart_mini_driver
でビルドしないとシェルスクリプトに実行権限が付与されない。

```
sudo chown -R root:root /home/colcon_ws
```
## プログラムの実行手順

### プログラムの実行の流れ

## Docker
### Optionの使用例 (GPUなし　コンテナ名=hokuyo_navigation2　共有フォルダ=/home/$USER/share)
```bash:bash
./docker/run.bash -n hokuyo_navigation2 -s /home/$USER/share
```

 ## コンテナ作成後
exitしてコンテナの外に出るとhomeディレクトリにCONTAINER_NAME.bash (CONTAINER_NAMEは自分で作成したコンテナの名前)が生成されている

```bash:bash
cd
./CONTAINER_NAME.bash
```
次回からは上記のスクリプトを実行すると自動でコンテナをスタートしてコンテナ内に入れる

# hokuyo_navigation2_gui

`hokuyo_navigation2_gui`は、ROS 2ベースのナビゲーションおよびマッピングシステムを直感的に操作するためのWebベースのGUIです。

Flaskサーバーを介して提供され、マッピング、ナビゲーションの実行、および関連するファイル（マップ、ウェイポイントなど）の管理をブラウザから簡単に行うことができます。

![画像](Image/hokuyo_navigation2_gui_pc.png)
