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

# x_ref = 1*data['time']
# y_ref = 1*data['time']

# x_ref = np.zeros_like(data['time'])
# y_ref = np.zeros_like(data['time'])
# for i, t in enumerate(time):
#     # if t > 40:
#     #     x_ref[i] = 40  
#     #     y_ref[i] = 40  
#     # else:
#         x_ref[i] = t
#         y_ref[i] = t

# yaw_ref = np.ones(data['time'].size) * (np.pi / 4)*180/np.pi

x_ref = 3*np.cos(np.pi/20*time)
y_ref = 3*np.sin(np.pi/20*time)
yaw_ref = (np.pi/2 + np.pi/20*time)*180/np.pi

v = data['v']
w = data['w']



plt.figure(1, figsize=(8, 4))
plt.plot(time, x_ref-x, label='error x vs time', color='g')
plt.title('Plot of error x vs time')
plt.xlabel('Time')
plt.ylabel('Error X')
plt.grid(True)
plt.legend()

plt.figure(2, figsize=(8, 4))
plt.plot(time, y_ref-y, label='error y vs time', color='g')
plt.title('Plot of error y vs time')
plt.xlabel('Time')
plt.ylabel('Error Y')
plt.grid(True)
plt.legend()

plt.figure(3, figsize=(8, 4))
plt.plot(time, yaw_ref-yaw, label='error yaw vs time', color='g')
plt.title('Plot of error yaw vs time')
plt.xlabel('Time')
plt.ylabel('Error Yaw')
plt.grid(True)
plt.legend()

plt.figure(figsize=(8, 8))
plt.subplot(2, 1, 1) 
plt.plot(time, v, label='forward speed vs time', color='b')
plt.title('Plot of forward speed vs time')
plt.xlabel('Time')
plt.grid(True)
plt.legend()
plt.subplot(2, 1, 2)
plt.plot(time, w, label='angular speed vs time', color='b')
plt.title('Plot of angular speed vs time')
plt.xlabel('Time')
plt.grid(True)
plt.legend()

plt.tight_layout()  # 自动调整子图间距

plt.show()

plt.figure(figsize=(8, 8))
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
