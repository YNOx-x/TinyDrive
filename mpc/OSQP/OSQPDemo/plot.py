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
yaw = data['theta']

v = data['v']
w = data['w']

e_x = data['e_x']
e_y = data['e_y']

plt.figure(1, figsize=(8, 4))
plt.plot(time, e_x, label='error x vs time', color='g')
plt.title('Plot of error x vs time')
plt.xlabel('Time')
plt.ylabel('Error X')
plt.grid(True)
plt.legend()

plt.figure(2, figsize=(8, 4))
plt.plot(time, e_y, label='error y vs time', color='g')
plt.title('Plot of error y vs time')
plt.xlabel('Time')
plt.ylabel('Error Y')
plt.grid(True)
plt.legend()




plt.show()


