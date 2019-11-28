import matplotlib.pyplot as plt
import csv

#for appending kalman filter and ground truth values
ekf_x = []
ekf_y = []

gt_x = []
gt_y = [];

#exception handling with "with"
with open('ekf_result.csv', 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter =',')
    for row in plots:
        ekf_x.append(float(row[0]))
        ekf_y.append(float(row[1]))

with open('ground_truth.csv', 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter =',')
    for row in plots:
        gt_x.append(float(row[0]))
        gt_y.append(float(row[1]))

plt.plot(ekf_x,ekf_y, label='EKF', linestyle = 'dotted',linewidth = 2)
plt.plot(gt_x,gt_y, label='Ground Truth', linewidth = 1)
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Kalman filter results')
plt.legend()
plt.show()
