#!/usr/bin/env python
import argparse
from rosbag import Bag
parser = argparse.ArgumentParser(description='Patch ITRI datasets.')
parser.add_argument('infile', metavar='in.bag')
parser.add_argument('lidar_input_topic')
args = parser.parse_args()
outfile = args.infile[:-4] + "-patched.bag"
print("Writing patched bag file to " + outfile)

smooth_queue_x_ang_vel = []
smooth_queue_y_ang_vel = []
smooth_queue_z_ang_vel = []
smooth_queue_x_lin_acc = []
smooth_queue_y_lin_acc = []
smooth_queue_z_lin_acc = []
smooth_queue_x_orient = []
smooth_queue_y_orient = []
smooth_queue_z_orient = []
smooth_queue_w_orient = []

smooth_queue_size = 100

good_gps_signal = False
imu_smoothing = False

with Bag(outfile, 'w') as fout:
    for topic, msg, t in Bag(args.infile):
        if topic == args.lidar_input_topic:
            msg.header.frame_id = 'velodyne'
            fout.write(topic, msg, t)
        if topic == '/nmea_sentence':
            if msg.sentence.split(',')[0] == '$GPGGA' and msg.sentence.split(',')[6] >= '4':
                good_gps_signal = True
            else:
                good_gps_signal = False
        if topic == '/fix_vs330':
            msg.header.frame_id = 'base_gps'
            if good_gps_signal == False:
                msg.status.status = -1
            fout.write("/fix", msg, t)
        if topic == '/imu/data' or topic == '/imu/data/':
            msg.header.frame_id = 'base_imu'

            if imu_smoothing == True:
                if len(smooth_queue_x_ang_vel) >= smooth_queue_size:
                    smooth_queue_x_ang_vel.pop(0)
                    smooth_queue_y_ang_vel.pop(0)
                    smooth_queue_z_ang_vel.pop(0)
                    smooth_queue_x_lin_acc.pop(0)
                    smooth_queue_y_lin_acc.pop(0)
                    smooth_queue_z_lin_acc.pop(0)
                    smooth_queue_x_orient.pop(0)
                    smooth_queue_y_orient.pop(0)
                    smooth_queue_z_orient.pop(0)
                    smooth_queue_w_orient.pop(0)

                smooth_queue_x_ang_vel.append(msg.angular_velocity.x)
                smooth_queue_y_ang_vel.append(msg.angular_velocity.y)
                smooth_queue_z_ang_vel.append(msg.angular_velocity.z)
                smooth_queue_x_lin_acc.append(msg.linear_acceleration.x)
                smooth_queue_y_lin_acc.append(msg.linear_acceleration.y)
                smooth_queue_z_lin_acc.append(msg.linear_acceleration.z)
                smooth_queue_x_orient.append(msg.orientation.x)
                smooth_queue_y_orient.append(msg.orientation.y)
                smooth_queue_z_orient.append(msg.orientation.z)
                smooth_queue_w_orient.append(msg.orientation.w)

                msg.angular_velocity.x = sum(smooth_queue_x_ang_vel) / len(smooth_queue_x_ang_vel)
                msg.angular_velocity.y = sum(smooth_queue_y_ang_vel) / len(smooth_queue_y_ang_vel)
                msg.angular_velocity.z = sum(smooth_queue_z_ang_vel) / len(smooth_queue_z_ang_vel)
                msg.linear_acceleration.x = sum(smooth_queue_x_lin_acc)/len(smooth_queue_x_lin_acc)
                msg.linear_acceleration.y = sum(smooth_queue_y_lin_acc)/len(smooth_queue_y_lin_acc)
                msg.linear_acceleration.z = sum(smooth_queue_z_lin_acc)/len(smooth_queue_z_lin_acc)
                msg.orientation.x = sum(smooth_queue_x_orient)/len(smooth_queue_x_orient)
                msg.orientation.y = sum(smooth_queue_y_orient)/len(smooth_queue_y_orient)
                msg.orientation.z = sum(smooth_queue_z_orient)/len(smooth_queue_z_orient)
                msg.orientation.w = sum(smooth_queue_w_orient)/len(smooth_queue_w_orient)

            fout.write(topic, msg, t)
