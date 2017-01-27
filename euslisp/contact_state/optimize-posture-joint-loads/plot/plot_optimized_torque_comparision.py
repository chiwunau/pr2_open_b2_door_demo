import rospy
import time
import numpy as np
import math
import matplotlib.pyplot as plt
import glob

file_r_org = glob.glob("../2016-11-13/*_r_*_org.txt")
file_r_opt = glob.glob("../2016-11-13/*_r_*_opt.txt")
file_l_org = glob.glob("../2016-11-13/*_r_*_org.txt")
file_l_opt = glob.glob("../2016-11-13/*_r_*_opt.txt")
file_norm_r_ratio = "../2016-11-13/_r_norm_ratio.txt"
file_norm_l_ratio = "../2016-11-13/_r_norm_ratio.txt"


name_r = []
name_l = []

max_tq_r_org = []
max_tq_r_opt = []
max_tq_l_org = []
max_tq_l_opt = []

tq_data_r_org = []
tq_data_r_opt = []

norm_r_ratio = []
norm_l_ratio = []

file_r_org.sort()
file_r_opt.sort()
for file in file_r_org:
    tq = []
    with open(file) as f:
        name_r.append(f.readline().split("\n")[0])
        max_tq_r_org.append(float(f.readline()))
        for line in f.readlines():
            # print line
            tq.append(float(line))
    tq_data_r_org.append(tq)

for file in file_r_opt:
    tq = []
    with open(file) as f:
         #name_r.append(f.readline().split("\n")[0])
        f.readline()
        max_tq_r_opt.append(float(f.readline()))
        for line in f.readlines():
            # print line
            tq.append(float(line))
    tq_data_r_opt.append(tq)

with open(file_norm_r_ratio) as f:
    for line in f.readlines():
        norm_r_ratio.append(float(line))

def draw():
    plt.figure(1)
    c = 0
    for i in range(len(name_r)):
        if not name_r[i] == "norm":
            plt.subplot(4,2,c+1)
            plt.ylim([0, 1 + max(max(map(abs,tq_data_r_org[i])), max(map(abs,tq_data_r_opt[i])), max_tq_r_org[i])])
            # plt.ylabel('Joint torque [Nm]')
            # plt.xlabel('calculation loop count')
            plt.plot(map(abs,tq_data_r_org[i]), "rx")
            plt.plot(map(abs,tq_data_r_opt[i]), "gx")
            plt.plot(map(abs,tq_data_r_org[i]), "r-")
            plt.plot(map(abs,tq_data_r_opt[i]), "g-")

            plt.title(name_r[i])
            plt.axhline(y=max_tq_r_org[i],xmin=0,xmax=100,color='k')#axhline(y=max_tq)
            c = i + 1
        else:
            plt.subplot(4,2,8)
            plt.ylim([0, 1 + max(max(tq_data_r_org[i]), max(tq_data_r_opt[i]))])
            plt.ylabel('Absolute joint torque [Nm]')
            plt.xlabel('Robot pose index')
            plt.plot(map(abs,tq_data_r_org[i]), "rx")
            plt.plot(map(abs,tq_data_r_opt[i]), "gx")
            plt.plot(map(abs,tq_data_r_org[i]), "r-")
            plt.plot(map(abs,tq_data_r_opt[i]), "g-")
            plt.title(name_r[i])

    ax2 = plt.subplot(4,2,8).twinx()
    ax2.set_ylabel("Ratio of torque change", color="b")
    ax2.axhline(y=1,xmin=0,xmax=100,linestyle="--",color='b')#axhline(y=max_tq)
    ax2.plot(norm_r_ratio, "bx")
    


    plt.show()
