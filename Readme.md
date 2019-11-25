# Kalman filter Implementation in C++ using Eigen. 

## This filter is used to localize a target using distance measurements from four satellites (modelled on UWB). 

![Screenshot of console](https://i.imgur.com/VipKiis.png?1raw=true "Screenshot of console")
## Current features:
- Create non-linear S-Shaped trajectory for target (Ground Truth)
- Generate distance measurements (with or without noise) for the four satellites
- The following parameters are editable: t (time step), measurement noise, max velocity of target and number of iterations
- An extended Kalman filter (EKF) is used in this case
- The user can input the starting X and Y positions in the console
- The final positions are shown in the console as well as the Grouth Truth values

## Features in development:
- Generate CSV file for KF positions at each time step 
- Read this CSV file with Python and create plots (trajectories with RMSE) 
- Implement a least squares solver and compare EKF with LSQ 
- Implement unscented Kalman filter and compare to EKF 
- Integrate vehicle model and generate system input for reconstructing the orientation of a vehicle 

## Planned features:
- Multiple targets that interact with each other for cooperative localization


References:
1. https://eigen.tuxfamily.org/dox/index.html
2. https://github.com/hmartiro/kalman-cpp 

*The two documents present under literature are written by me. Please ask for permission before reusing them.*
contact: ankursrivastava.ansr@gmail.com 
