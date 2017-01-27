import rospy
import time
import numpy as np
import math
import matplotlib.pyplot as plt

diff_p = []
diff_b = []

with open("diff-p.txt") as f:
    for line in f.readlines():
        print line
        diff_p.append(float(line))

with open("diff-b.txt") as f:
    for line in f.readlines():
        print line
        diff_b.append(float(line))

def draw():
    plt.plot(diff_p, color="red", label="diff-p")
    plt.plot(diff_b, color="blue", label="diff-b")
    plt.show()
