#!/usr/bin/env python3
# Generate csv from rosbag file

import rosbag
import rospy
import sys
import os
import csv
import math

args = sys.argv

# get path
filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))
output_csv=os.path.normpath(os.path.join(os.getcwd(),args[2]))
thre=math.sqrt(float(args[3]))

# read the bag file
global data_num
global sumx, sumy, sumz
global count
global x_avesum
global y_avesum
global z_avesum

global x_ave
global y_ave
global z_ave

count = 0
sumx = 0
sumy = 0
sumz = 0

x_ave = 0
y_ave = 0
z_ave = 0

x_avesum = 0
y_avesum = 0
z_avesum = 0

bag = rosbag.Bag(filename)

with open(output_csv, 'w') as csvfile:
    filewriter = csv.writer(csvfile, delimiter = ',')
    header = ["北方向のばらつき[m]" , "東方向のばらつき[m]", "鉛直方向のばらつき[m]"]
    filewriter.writerow(header)
    
    for topic, msg, t in bag.read_messages(topics=['/fix']):
        p = msg.position_covariance
        sumx += math.sqrt(p[0])
        sumy += math.sqrt(p[4])
        sumz += math.sqrt(p[8])
        if math.sqrt(p[0]) < thre:
            x_avesum += 1
        if math.sqrt(p[4]) < thre:
            y_avesum += 1
        if math.sqrt(p[8]) < thre:
            z_avesum += 1
        count += 1
        ax = sumx/count
        ay = sumy/count
        az = sumz/count
        x_ave=x_avesum/count*100
        y_ave=y_avesum/count*100
        z_ave=z_avesum/count*100
        filewriter.writerow([math.sqrt(p[0]), math.sqrt(p[4]), math.sqrt(p[8])])
    
with open(output_csv, "r") as csvfile:
    lines = csvfile.readlines()

a = str(ax)+','+str(ay)+','+str(az)+'\n'
lines.insert(0,'北方向のばらつきの平均[m], 東方向のばらつきの平均[m], 鉛直方向のばらつきの平均[m]\n')
lines.insert(1,a)

b = str(x_ave)+','+str(y_ave)+','+str(z_ave)+'\n'
lines.insert(0,'北方向のfix率[%],東方向のfix率[%],鉛直方向のfix率[%]\n')
lines.insert(1,b)

with open(output_csv, "w") as csvfile:
    csvfile.writelines(lines)

print("北方向のfix率, 東方向のfix率, 鉛直方向のfix率")
print(x_ave,y_ave,z_ave)
print("北方向のばらつきの平均[m], 東方向のばらつきの平均[m], 鉛直方向のばらつきの平均[m]")
print(ax,ay,az)

bag.close()