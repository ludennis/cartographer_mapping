#!/usr/bin/python

import sys
import csv

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Usage: trajectory_plotter [csv_filename]"
        sys.exit(-1)

    csv_filename = sys.argv[1]
    csv_file = open(csv_filename, "r")
    csv_reader = csv.reader(csv_file)

    xx, yy, zz = [], [], []
    for i, row in enumerate(csv_reader):
        if i > 0:
            xx.append(float(row[2]))
            yy.append(float(row[3]))
            zz.append(float(row[4]))

    fig = plt.figure()
    ax = fig.gca(projection="3d")
    ax.plot(xs=xx, ys=yy, zs=zz)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    ax.xaxis._axinfo["grid"]["color"] = "#808080"
    ax.yaxis._axinfo["grid"]["color"] = "#808080"
    ax.zaxis._axinfo["grid"]["color"] = "#808080"

    plt.show()
