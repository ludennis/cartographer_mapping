#!/usr/bin/env python2

import numpy as np
import csv
import matplotlib.pyplot as plt
import sys

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print "Usage: rosrun bag_processor pose_plotter.py [predict_pose csv file]" \
            " [predict_pose_smoothed csv file]"
        sys.exit()

    for i, arg in enumerate(sys.argv):
        print "argument {}: {}".format(i, arg)

    # load predict_pose csv file
    print "Opening predict_pose csv file: {}".format(sys.argv[1])
    with open(sys.argv[1], 'r') as csv_file:
        data_iter = csv.reader(csv_file, delimiter=',')
        data = [row for row in data_iter]
        header = data[0]
        data = data[1:]
    data = np.array(data)
    print header

    for i, header in enumerate(header):
        if header == 'pose/position/x':
            predict_pose_x = data[:, i]
        elif header == 'pose/position/y':
            predict_pose_y = data[:, i]
        elif header == 'time':
            predict_pose_time = data[:, i]
        elif header == 'header/stamp/nsecs':
            predict_pose_nsecs = data[:, i]
        elif header == 'header/stamp/secs':
            predict_pose_secs = data[:, i]

    # load predict_pose_smoothed csv file
    print "Opening predict_pose_smoothed csv file: {}".format(sys.argv[2])
    with open(sys.argv[2], 'r') as csv_file:
        data_iter = csv.reader(csv_file, delimiter=',')
        data = [row for row in data_iter]
        header = data[0]
        data = data[1:]
    data = np.array(data)
    print header

    for i, header in enumerate(header):
        if header == 'pose/position/x':
            predict_pose_smoothed_x = data[:, i]
        elif header == 'pose/position/y':
            predict_pose_smoothed_y = data[:, i]
        elif header == 'time':
            predict_pose_smoothed_time = data[:, i]
        elif header == 'header/stamp/nsecs':
            predict_pose_smoothed_nsecs = data[:, i]
        elif header == 'header/stamp/secs':
            predict_pose_smoothed_secs = data[:, i]

    print 'There are {} of x coordinates, and {} of y coordinates in predict_pose'.format( \
        len(predict_pose_x), len(predict_pose_y))

    print 'There are {} of x coordinates, and {} of y coordinates in predict_pose_smoothed'.format( \
        len(predict_pose_smoothed_x), len(predict_pose_smoothed_y))

    # plot both predcit_pose and predict_pose_smoothed
    fig, ax = plt.subplots()
    predict_pose_line, = plt.plot(predict_pose_x, predict_pose_y, 'r.-')
    predict_pose_smoothed_line, = plt.plot(predict_pose_smoothed_x, predict_pose_smoothed_y, 'b.-')

    plt.show()
