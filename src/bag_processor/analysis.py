import json
import numpy as np
import matplotlib.pyplot as plt

with open('data/detected_objects.json') as json_file:
    objects = json.load(json_file)

with open('data/x.json') as json_file:
    x = json.load(json_file)

with open('data/vehicle_state.json') as json_file:
    vehicleState = json.load(json_file)

tCmd = list()
speedCmd = list()
for i in x['x']:
    speedCmd.append(i['x'])
    tCmd.append(i['time'])
tCmd = np.array(tCmd)
speedCmd = np.array(speedCmd)

tFb = list()
speedFb = list()
autoMode = list()
for i in vehicleState['vehicle_state']:
    speedFb.append(i['speed'])
    tFb.append(i['time'])
    autoMode.append(i['auto_mode'])
tFb = np.array(tFb)
speedFb = np.array(speedFb)
autoMode = np.array(autoMode)

aaa = tCmd[np.where( speedCmd == 0.0 )]
bbb = speedCmd[np.where( speedCmd == 0.0 )]

tObj = list()
for i in objects['detected_objects']:
    tObj.append(i['time'])
tObj = np.array(tObj)


plt.plot(tCmd, speedCmd)
plt.plot(aaa, bbb)
# plt.plot(tFb, speedFb)
# plt.plot(tFb, autoMode)
plt.show()
