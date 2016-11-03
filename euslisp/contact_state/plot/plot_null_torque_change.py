import rospy
import time
import numpy as np
import math
import matplotlib.pyplot as plt
import glob

file_list = glob.glob("../test_torque/*.txt")
name = []
max_tq = []
tq_data = []

for file in file_list:
    tq = []
    with open(file) as f:
        name.append(f.readline().split("\n")[0])
        max_tq.append(float(f.readline()))
        for line in f.readlines():
            # print line
            tq.append(float(line))
    tq_data.append(tq)

def draw():
    plt.figure(1)
    for i in range(len(name)):
        plt.subplot(4,2,i+1)
        plt.ylim([0, 5 + max(max(tq_data[i]), max_tq[i])])
        plt.plot(map(abs,tq_data[i]), "k")
        plt.title(name[i])
        plt.axhline(y=max_tq[i],xmin=0,xmax=100,color='r')#axhline(y=max_tq)

    plt.show()
