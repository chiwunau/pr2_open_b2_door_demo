import rospy
import time
import numpy as np
import math
import matplotlib.pyplot as plt
import glob

file_r_org = glob.glob("../2016-11-12/*_r_*_org.txt")
file_r_opt = glob.glob("../2016-11-12/*_r_*_opt.txt")
file_l_org = glob.glob("../2016-11-12/*_l_*_org.txt")
file_l_opt = glob.glob("../2016-11-12/*_l_*_opt.txt")

name_r = []
name_l = []

max_tq_r_org = []
max_tq_r_opt = []
max_tq_l_org = []
max_tq_l_opt = []

tq_data_r_org = []
tq_data_r_opt = []

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

def draw():
    plt.figure(1)
    c = 0
    for i in range(len(name_r)):
        if not name_r[i] == "norm":
            plt.subplot(4,2,c+1)
            plt.ylim([0, 1 + max(max(map(abs,tq_data_r_org[i])), max(map(abs,tq_data_r_opt[i])), max_tq_r_org[i])])
            # plt.ylabel('Joint torque [Nm]')
            # plt.xlabel('calculation loop count')
            plt.plot(map(abs,tq_data_r_org[i]), "ro")
            plt.plot(map(abs,tq_data_r_opt[i]), "g^")
            plt.plot(map(abs,tq_data_r_org[i]), "r--")
            plt.plot(map(abs,tq_data_r_opt[i]), "g--")

            plt.title(name_r[i])
            plt.axhline(y=max_tq_r_org[i],xmin=0,xmax=100,color='k')#axhline(y=max_tq)
            c = i + 1
        else:
            plt.subplot(4,2,8)
            plt.ylim([0, 1 + max(max(tq_data_r_org[i]), max(tq_data_r_opt[i]))])
            plt.ylabel('Absolute joint torque [Nm]')
            plt.xlabel('Robot pose index')
            plt.plot(map(abs,tq_data_r_org[i]), "ro")
            plt.plot(map(abs,tq_data_r_opt[i]), "g^")
            plt.plot(map(abs,tq_data_r_org[i]), "r--")
            plt.plot(map(abs,tq_data_r_opt[i]), "g--")
            plt.title(name_r[i])
    plt.show()
