# -- coding: utf-8 --
import sys
import os
import numpy as np

# コマンドライン引数 引数1：入力ファイル(path) 引数2：出力ファイル名(path) 引数3：p2o出力ファイル(path) 引数4：初期ポーズ出力ファイル(path) 引数5：初期緯度経度高度出力ファイル(path)
args = sys.argv
input_file = os.path.normpath(os.path.join(os.getcwd(), args[1]))
output_file = os.path.normpath(os.path.join(os.getcwd(), args[2]))
out_p2o = os.path.normpath(os.path.join(os.getcwd(), args[3]))
initial_pose = os.path.normpath(os.path.join(os.getcwd(), args[4]))
initial_lat_lon_alt = os.path.normpath(os.path.join(os.getcwd(), args[5]))

# --------------------------------------------------------------------------------------
# p2o ファイルから原点座標と初期位置情報を読み込む
# --------------------------------------------------------------------------------------
try:
    with open(out_p2o, "r") as f:
        p2o_lines = f.readlines()
        if len(p2o_lines) < 2:
            print("エラー：p2o ファイルの行数が不足しています。")
            sys.exit(1)
        p2o_data = np.array([list(map(float, line.split())) for line in p2o_lines])
        if p2o_data.shape[0] < 2 or p2o_data.shape[1] < 7:
            print("エラー：p2o ファイルのデータ形式が不正です。")
            sys.exit(1)
        
        # p2o_origin を点群の新しい原点として使用します
        p2o_origin = p2o_data[1, :3]
        initial_lat_lon_alt_data = p2o_data[1, 7:]
        initial_quat = p2o_data[1, 3:7]

except FileNotFoundError as err:
    print(err)
    print('p2o ファイルが見つかりません。')
    sys.exit(1)
except ValueError as err:
    print(err)
    print('p2o ファイルの数値形式が不正です。')
    sys.exit(1)
except IndexError as err:
    print(err)
    print('p2o ファイルの形式が想定外です。')
    sys.exit(1)

# --------------------------------------------------------------------------------------
# input_file から点群データを読み込み、平行移動を実行
# --------------------------------------------------------------------------------------
try:
    with open(input_file, "r") as f:
        header_lines = [next(f) for _ in range(11)]
        
        # input_file 内の原点（12行目）を読み込みます
        origin_line = next(f).split()
        if origin_line:
            input_origin = np.array(list(map(float, origin_line)))
        else:
            print("エラー：入力ファイルの原点の行が空です。")
            sys.exit(1)
            
        # 残りのデータを一括で読み込みます
        data = np.loadtxt(f)
        
except FileNotFoundError as err:
    print(err)
    print('入力ファイルが見つかりません。')
    sys.exit(1)
except ValueError as err:
    print(err)
    print('入力ファイルの数値形式が不正です。')
    sys.exit(1)
except StopIteration:
    print("エラー：入力ファイルの形式が不正です。原点の行が見つかりません。")
    sys.exit(1)

print("点群の平行移動を開始します。")

# p2o の原点座標を引いて、点群全体を移動させます
# input_file の原点座標も同様に移動させます
translated_origin = input_origin - p2o_origin
translated_data = data - p2o_origin

# 新しい原点と移動した点群データを結合します
calculated_data = np.vstack([translated_origin, translated_data])

print("点群の平行移動を終了しました。")

# --------------------------------------------------------------------------------------
# 結果をファイル出力
# --------------------------------------------------------------------------------------
with open(output_file, "w") as f:
    f.writelines(header_lines)
    np.savetxt(f, calculated_data, fmt='%f')

# 初期位置の計算とファイル出力
# initial_translation は p2o_origin を引いた後の input_origin の位置になります
initial_translation = input_origin - input_origin
initial_pose_data = np.concatenate([initial_translation, initial_quat])

with open(initial_pose, "w") as f:
    f.write(",".join(map(str, initial_pose_data)) + "\n")

with open(initial_lat_lon_alt, "w") as f:
    f.write(",".join(map(str, initial_lat_lon_alt_data)) + "\n")

# デバッグ用出力
if 'DEBUG' in os.environ:
    print("p2o_origin:")
    print(p2o_origin)
    print("input_origin:")
    print(input_origin)
    print("Translated Origin:")
    print(translated_origin)
    print("Translated Data (first 5):")
    print(calculated_data[:6])
    print(f"Shape of Calculated Data: {calculated_data.shape}")