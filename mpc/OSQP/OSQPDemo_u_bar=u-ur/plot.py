import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

file_path = "./record_data.csv"
if os.path.exists(file_path):
    data = pd.read_csv(file_path)
else:
    data = pd.read_csv("./build/record_data.csv")

time = data['time']
x = data['x']
y = data['y'] 
yaw = data['yaw']

left_rpm = data['left_rpm']
right_rpm = data['right_rpm']

# reference path: circle
# x_ref = np.zeros_like(data['time'])
# y_ref = np.zeros_like(data['time'])
# for i, t in enumerate(time):
#     x_ref[i] = t
#     y_ref[i] = t

# yaw_ref = np.ones(data['time'].size) * (np.pi / 4)*180/np.pi


# # reference path: line
# x_ref = 3*np.cos(np.pi/20*time)
# y_ref = 3*np.sin(np.pi/20*time)
# yaw_ref = (np.pi/2 + np.pi/20*time)*180/np.pi


# reference path: curve
x_ref = np.zeros_like(data['time'])
y_ref = np.zeros_like(data['time'])
yaw_ref = np.zeros_like(data['time'])
for i, t in enumerate(time):
    x_ref[i] = t
    y_ref[i] = 3 * np.cos(t * np.pi/20)
    dxref = 1.0
    dyref = -3 * np.pi/20 * np.sin(t * np.pi/20)
    yaw_ref[i] = np.arctan2(dyref, dxref) * 180/np.pi


v = data['v']
w = data['w']

e_x = data['e_x']
e_y = data['e_y']


plt.figure(1, figsize=(8, 8))
plt.subplot(3, 1, 1)
plt.plot(time, x_ref-x, label='error x vs time', color='g')
plt.title('Plot of error x vs time')
plt.xlabel('Time')
plt.ylabel('Error X')
plt.grid(True)
plt.legend()
plt.subplot(3, 1, 2)
plt.plot(time, y_ref-y, label='error y vs time', color='g')
plt.title('Plot of error y vs time')
plt.xlabel('Time')
plt.ylabel('Error Y')
plt.grid(True)
plt.legend()
plt.subplot(3, 1, 3)
plt.plot(time, yaw_ref-yaw, label='error yaw vs time', color='g')
plt.title('Plot of error yaw vs time')
plt.xlabel('Time')
plt.ylabel('Error Yaw')
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.figure(2, figsize=(8, 8))
plt.subplot(2, 1, 1) 
plt.plot(time, e_x, label='e_x vs time', color='b')
plt.title('Plot of e_x vs time')
plt.xlabel('Time')
plt.grid(True)
plt.legend()
plt.subplot(2, 1, 2)
plt.plot(time, e_y, label='e_y vs time', color='b')
plt.title('Plot of e_y vs time')
plt.xlabel('Time')
plt.grid(True)
plt.legend()
plt.tight_layout() 


# plt.figure(3, figsize=(8, 8))
# plt.subplot(2, 1, 1) 
# plt.plot(time, v, label='forward speed vs time', color='b')
# plt.title('Plot of forward speed vs time')
# plt.xlabel('Time')
# plt.grid(True)
# plt.legend()
# plt.subplot(2, 1, 2)
# plt.plot(time, w, label='angular speed vs time', color='b')
# plt.title('Plot of angular speed vs time')
# plt.xlabel('Time')
# plt.grid(True)
# plt.legend()
# plt.tight_layout() 

plt.figure(4, figsize=(8, 4))
plt.plot(time, left_rpm, label='left velocity vs time', color='b')
plt.plot(time, right_rpm, label='right velocity vs time', color='r')
plt.title('Plot of left and right velocity vs time')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.grid(True)
plt.legend()

plt.show()

plt.figure(5, figsize=(8, 8))
plt.subplot(3, 1, 1) 
plt.plot(time, x_ref, label='x_ref vs time', color='r')
plt.plot(time, x, label='x vs time', color='b')
plt.title('Plot of x and x_ref vs time')
plt.xlabel('Time')
plt.grid(True)
plt.legend()
plt.subplot(3, 1, 2)
plt.plot(time, y_ref, label='y_ref vs time', color='r')
plt.plot(time, y, label='y vs time', color='b')
plt.title('Plot of y and y_ref vs time')
plt.xlabel('Time')
plt.grid(True)
plt.legend()
plt.subplot(3, 1, 3)
plt.plot(time, yaw_ref, label='yaw_ref vs time', color='r')
plt.plot(time, yaw, label='yaw vs time', color='b')
plt.title('Plot of yaw and yaw_ref vs time')
plt.xlabel('Time')
plt.grid(True)
plt.legend()

plt.tight_layout()  # 自动调整子图间距
plt.show()