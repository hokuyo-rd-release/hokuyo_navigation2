# -- coding: utf-8 --
import sys
import os

# コマンドライン引数 引数1：入力ファイル(path) 引数2：出力ファイル名(path)
args = sys.argv
input_file=os.path.normpath(os.path.join(os.getcwd(),args[1]))
output_file=os.path.normpath(os.path.join(os.getcwd(),args[2]))
out_p2o = os.path.normpath(os.path.join(os.getcwd(),args[3]))
initial_pose=os.path.normpath(os.path.join(os.getcwd(),args[4]))
initial_lat_lon_alt=os.path.normpath(os.path.join(os.getcwd(),args[5]))

# ファイルの読み込み先頭から1行ずつ読み込む
with open(input_file, "r") as f:
    try:
        lines = f.readlines()
    except FileNotFoundError as err:
        print(err)
        print('点群のトピック名が間違っています。')

# 初期位置の取得
# output.p2o_out.txt
with open(out_p2o, "r") as f:
    try:
        p2o = f.readlines()
    except FileNotFoundError as err:
        print(err)
        print('file not found.')

# p2o ファイルの並進移動量を格納する。
# p2ox_y_z = []
# ox, oy, oz, p2oq1_dummy, p2oq2_dummy, p2oq3_dummy, p2oq4_dummy, lat, lon, alt = map(float, p2o[10].split())
ox, oy, oz, p2oq1, p2oq2, p2oq3, p2oq4, lat, lon, alt = map(float, p2o[1].split())
# p2ox_y_z.append([p2ox, p2oy, p2oz])

# 原点を配列に導入する。
ox_y_z = []
ox_y_z.append([ox, oy, oz])
oxyz = []
o_x,o_y,o_z = map(float, lines[11].split())
oxyz.append([o_x,o_y,o_z])

# 計算に用いる配列を記録する。
x_y_z = []
for i in range(12, len(lines)):
    x,y,z = map(float, lines[i].split())
    x_y_z.append([x,y,z])

# 座標の計算結果を格納する配列を確保 x_y_z + 1(原点)と同じ長さの配列
cx_y_z = [[] for _ in range(len(x_y_z)+1)]

# リスト同士の計算 内包表記
cx_y_z[0] = [x - y for x, y in zip(oxyz[0], ox_y_z[0])]
for i in range(0, len(x_y_z)):
    cx_y_z[i+1] = [x - y for x, y in zip(x_y_z[i], ox_y_z[0])]

# ファイル出力
with open(output_file, "w") as f:
    # 文字列部
    for i in range(0,11):
        f.writelines(lines[i])
    # データ部を読み込み
    for row in cx_y_z:
        f.write(" ".join(map(str, row)) + "\n")


initx_y_z = []
initx = ox - ox
inity = oy - oy
initz = oz - oz
initx_y_z.append([initx,inity,initz,p2oq1,p2oq2,p2oq3,p2oq4])

init_lat_lon_alt = []
init_lat_lon_alt.append([lat,lon,alt])

with open(initial_pose, "w") as f:
    for row in initx_y_z:
        f.write(",".join(map(str, row)) + "\n")

with open(initial_lat_lon_alt,"w") as f:
    for row in init_lat_lon_alt:
        f.write(",".join(map(str, row)) + "\n")

# 出力確認 list 10個出力
# for i in range(0, 5):
#     print(x_y_z[i-1])
#     print(ox_y_z[0])
#     print(cx_y_z[i])
# print(len(cx_y_z))