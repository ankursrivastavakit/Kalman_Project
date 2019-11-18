#Kalman filter Implementation in C++ using Eigen. 

##This filter is used to localize a target using distance measurements from four satellites. 

##Current features:
- Create non-linear S-Shaped trajectory for target (Ground Truth)
- Generate distance measurements (with or without noise) for the four satellites
- The following parameters are editable: t (time step), measurement noise, max velocity of target and number of iterations
- An extended Kalman filter (EKF) is used in this case
- The user can input the starting X and Y positions in the console
- The final positions are shown in the console as well as the ground truth values

##Features in development with planned completion date:
- Generate CSV file for KF positions at each time step (20.11.19)
- Read this CSV file with Python and create a plot ( 20.11.19)
- Implement a least squares solver and compare EKF with LSQ (21.11.19)
- Implement unscented Kalman filter and compare to EKF (23.11.19)

##Planned features:
- Integrate vehicle model and generate system input for reconstructing the orientation of a vehicle (30.11.19)
- Multiple targets that interact with each other for cooperative localization
