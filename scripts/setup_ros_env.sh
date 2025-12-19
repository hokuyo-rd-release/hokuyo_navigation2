#!/bin/bash
# --------------------------------------------------------------------------
# setup_ros_env.sh: ROS 2 環境設定を共通化するスクリプト
#
# このスクリプトは、呼び出し元のシェルのカレントディレクトリを変更せず、
# 必要な環境変数を設定します。
# --------------------------------------------------------------------------

# bash/zsh互換の方法で、このスクリプト自身のパスを取得する
# BASH_SOURCE[0] はbashで、${(%):-%x} はzshで有効
SCRIPT_PATH="${BASH_SOURCE[0]:-${(%):-%x}}"

# このスクリプトの場所から hokuyo_navigation2 パッケージのパスを特定
# (cd ... && pwd) を使うことで、シンボリックリンクを解決し、絶対パスを取得
# dirname "$SCRIPT_PATH" でスクリプトのディレクトリを取得
SCRIPT_DIR=$(cd "$(dirname "$SCRIPT_PATH")" && pwd)
HOKUYO_NAV2_PKG_PATH=$(cd "${SCRIPT_DIR}/.." && pwd)

# hokuyo_navigation2 パッケージのパスから ROS 2 ワークスペースのルートを特定
ROS2_WS=$(cd "${HOKUYO_NAV2_PKG_PATH}/../../.." && pwd)

# ROS 2 環境をセットアップ
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"
source ~/.bashrc
# source ~/.bash_spel_setting

# 変数をエクスポートしてサブシェルでも利用可能にする
export ROS2_WS HOKUYO_NAV2_PKG_PATH