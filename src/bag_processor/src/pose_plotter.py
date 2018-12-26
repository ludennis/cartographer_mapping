#!/usr/bin/env python2

import numpy as np
import csv
import matplotlib.pyplot as plt
import sys

def update_annotation(index):
    x, y = predict_pose_line.get_data()
    predict_pose_annotations.xy = (predict_pose_x[index["ind"][0]], predict_pose_y[index["ind"][0]])
    predict_pose_text = "predict_pose => point ({}, {}), index: {}, timestamp: {}".format(
        " ".join([predict_pose_x[n] for n in index["ind"]]),
        " ".join([predict_pose_y[n] for n in index["ind"]]),
        " ".join(list(map(str, index["ind"]))),
        " ".join([predict_pose_time[n] for n in index["ind"]]))
    predict_pose_annotations.set_text(predict_pose_text)
    predict_pose_annotations.get_bbox_patch().set_alpha(0.4)

    x, y = predict_pose_smoothed_line.get_data()
    predict_pose_smoothed_annotations.xy = (predict_pose_smoothed_x[index["ind"][0]],
        predict_pose_smoothed_y[index["ind"][0]])
    predict_pose_smoothed_text = "predict_pose_smoothed => point ({}, {}), index: {}, timestamp: {}".format(
        " ".join([predict_pose_smoothed_x[n] for n in index["ind"]]),
        " ".join([predict_pose_smoothed_y[n] for n in index["ind"]]),
        " ".join(list(map(str, index["ind"]))),
        " ".join([predict_pose_smoothed_time[n] for n in index["ind"]]))
    predict_pose_smoothed_annotations.set_text(predict_pose_smoothed_text)
    predict_pose_smoothed_annotations.get_bbox_patch().set_alpha(0.4)

def hover(event):
    predict_pose_annotation_is_visible = predict_pose_annotations.get_visible()
    predict_pose_smoothed_annotation_is_visible = predict_pose_smoothed_annotations.get_visible()
    if event.inaxes == ax:
        predict_pose_cont, predict_pose_index = predict_pose_line.contains(event)
        predict_pose_smoothed_cont, predict_pose_smoothed_index = predict_pose_smoothed_line.contains(event)
        if predict_pose_cont:
            update_annotation(predict_pose_index)
            predict_pose_annotations.set_visible(True)
            fig.canvas.draw_idle()
        elif predict_pose_smoothed_cont:
            update_annotation(predict_pose_smoothed_index)
            predict_pose_smoothed_annotations.set_visible(True)
            fig.canvas.draw_idle()
        else:
            if predict_pose_annotation_is_visible:
                predict_pose_annotations.set_visible(False)
                fig.canvas.draw_idle()
            elif predict_pose_smoothed_annotation_is_visible:
                predict_pose_smoothed_annotations.set_visible(False)
                fig.canvas.draw_idle()

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

    predict_pose_annotations = ax.annotate("", xy=(0, 0), xytext=(-20, 20), textcoords="offset points",
        bbox=dict(boxstyle="round", fc="w"), arrowprops=dict(arrowstyle="->"))

    predict_pose_smoothed_annotations = ax.annotate("", xy=(0, 0), xytext=(-20, 20), textcoords="offset points",
        bbox=dict(boxstyle="round", fc="w"), arrowprops=dict(arrowstyle="->"))

    predict_pose_annotations.set_visible(False)
    predict_pose_smoothed_annotations.set_visible(False)

    fig.canvas.mpl_connect("motion_notify_event", hover)

    plt.show()
