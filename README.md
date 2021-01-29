# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project I have utilized a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements with RMSE values that are lower than the certain tolerance.

(This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases))

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Vanilla Kalman filter , used for linear systems , is used in this project inorder to find an optimal estimate of the state of the toy car based on the Lidar and Radar measurement of the surroundings . Linear state space model of kalman filter can be used for lidar data but processing of radar data involves non linear function due to this initially radar mesaurements are linearly approximated using Jacoban matrices and then used in kalman filter algorithm for optimal state estimation. 

The following shows the output of the algorithm when ran , for only lidar data, for only radar data , when ran on the combined measurements data :

(Green curve is the collection of optimal state estimate points)

## Only Lidar Data : 

![](https://github.com/nikhilbadam56/EKF_Udacity/blob/master/PIctures/only_laser_output.png?raw=true)

## Only Radar Data

![](https://github.com/nikhilbadam56/EKF_Udacity/blob/master/PIctures/only_radar_output.png?raw=true)

## Combined

![](https://github.com/nikhilbadam56/EKF_Udacity/blob/master/PIctures/Output_dataset1.png?raw=true)
