#!/bin/bash

# プログレスバーを表示したい秒数を指定
TOTAL_SECONDS=$1
TITLE="Progress"

# Zenityでプログレスバーダイアログを表示
(
    # カウントダウンの開始時間を記録
    START_TIME=$(date +%s)
    
    # 終了時間を計算
    END_TIME=$((START_TIME + TOTAL_SECONDS))
    
    # ループの開始時間を取得
    CURRENT_TIME=$(date +%s)
    
    while [ "$CURRENT_TIME" -le "$END_TIME" ]; do
        # 経過時間を計算
        ELAPSED_TIME=$((CURRENT_TIME - START_TIME))
        
        # 残り時間を計算
        REMAINING_SECONDS=$((TOTAL_SECONDS - ELAPSED_TIME))
        
        # パーセンテージを計算（bcコマンドを使って浮動小数点計算）
        PERCENTAGE=$(echo "scale=2; ($ELAPSED_TIME * 100) / $TOTAL_SECONDS" | bc | cut -d'.' -f1)
        
        # Zenityに渡すテキストとパーセンテージを組み立て
        # `#`で始まる行はzenityのテキストが更新されます
        echo "#残り時間: ${REMAINING_SECONDS} 秒"
        echo "$PERCENTAGE"
        
        # 1秒待機
        sleep 1
        
        # 次のループのために現在時間を更新
        CURRENT_TIME=$(date +%s)
    done

    # 最後の更新
    echo "#rosbag処理中。残り時間: 0 秒"
    echo "100"
) | zenity --progress --title="$TITLE" --auto-close