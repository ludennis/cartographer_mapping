#!/usr/bin/env python2

import rosbag
import numpy as np
import os
import sys
import json
import csv
import matplotlib.pyplot as plt

def MsgToDictionary(msg, name, data):
    try:
        dataTmp = dict()
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            MsgToDictionary(val, str(s), dataTmp)
        data[name] = dataTmp
    except:
        if isinstance(msg, list):
            listTmp = list()
            dataTmp = dict()
            for m in msg:
                MsgToDictionary(m, name, dataTmp)
                listTmp.append(dataTmp[name])
            data[name] = listTmp
        else:
            data[name] = msg

def Flatten(d, parent_key='', sep='/'):
    items = []
    for k, v in d.items():
        new_key = parent_key + sep + k if parent_key else k
        try:
            items.extend(Flatten(v, new_key, sep=sep).items())
        except:
            items.append((new_key, v))
    return dict(items)

def GetFlatten(data, key):
    try:
        dataFlat = dict()
        for v in data[key]:
            flatten = Flatten(v)
            for kf, vf in flatten.items():
                if kf in dataFlat.keys():
                    dataFlat[kf].append(vf)
                else:
                    dataFlat[kf] = list()
    except:
        dataFlat = data

    return dataFlat

def DictToJson(data, topic):
    filename = 'data/' + topic + '.json'
    with open(filename, 'w') as outfile:
        json.dump(data, outfile, indent=4)

def DictToCsv(data, topic):
    filename = 'data/' + topic + '.csv'
    with open(filename, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(data.keys())
        dataCsv = zip(*data.values())
        for dataCsvRow in dataCsv:
            writer.writerow(list(dataCsvRow))
        f.close()

def Visualization(data):
    for k, v in data.items():
        try:
            if 'time' not in k:
                t = np.array(data['time'])
                plt.plot(t - data['time'][0], v, label=k)
        except:
            pass

def Analysis(topic, bag):
    if '_/' in topic:
        topicTmp = topic.split('_/')
        topicMember = topicTmp[-1].split('/')
        topicMember = filter(None, topicMember)
        topicMember[0] = topicTmp[0] + '/' + topicMember[0]
    else:
        topicMember = topic.split('/')
        topicMember = filter(None, topicMember)

    targetMember = 'msg'
    if len(topicMember) > 1:
        for member in topicMember[1:]:
            targetMember = targetMember + "." + member

    dataTmp = dict()
    data = dict()
    for topic, msg, t in bag.read_messages(topics='/' + topicMember[0]):
        MsgToDictionary(eval(targetMember), topicMember[-1], dataTmp)
        if topicMember[-1] in data.keys():
            if isinstance(dataTmp[topicMember[-1]], dict):
                dataTmp[topicMember[-1]]['time'] = t.to_sec()
                data[topicMember[-1]].append(dataTmp[topicMember[-1]])
            else:
                data[topicMember[-1]].append({'time': t.to_sec(), topicMember[-1]: dataTmp[topicMember[-1]]})
        else:
            data[topicMember[-1]] = list()

    dataFlat = GetFlatten(data, topicMember[-1])

    if '--flatten' in (argv for argv in sys.argv):
        DictToJson(dataFlat, topicMember[-1])
    else:
        DictToJson(data, topicMember[-1])

    DictToCsv(dataFlat, topicMember[-1])

    Visualization(dataFlat)

def main():
    directory = 'data'
    if not os.path.exists(directory):
        os.makedirs(directory)

    if len(sys.argv) < 3 :
        sys.stdout.write("\033[1;31m")
        print('Error, wrong arguments')
        sys.stdout.write("\033[0;32m")
        print('example:\n\n    rosrun bag_processor bag_processor.py [bag_file] [topic]\n')
        print(' or rosrun bag_processor bag_processor.py [bag_file] --list\n')
        sys.exit(1)

    print('reading bag file... ')
    print('    ' + sys.argv[1])
    bag = rosbag.Bag(sys.argv[1])
    topics = bag.get_type_and_topic_info()[1].keys()

    if '--list' in (argv for argv in sys.argv):
        print('\ntopics:\n')
        for topic in topics:
            print('    ' + topic)
    else:
        for topic in sys.argv[2:]:
            Analysis(topic, bag)
        plt.xlabel('time(sec)')
        plt.legend()
        plt.show()

    bag.close()

if __name__ == '__main__':
    main()
