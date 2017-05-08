# Unscented-Kalman-Filter-Project
Udacity's Self-Driving Car Engineer Nanodegree Program Project.

---

## Abstract
Tracking systems allow autonomous vehicles to safely drive when obstacles are present, and that is particularly interesting when pedestrians, bicycles or motorbikes are close to the car. Is in these situations where having a huge precision is critical.
Therefore, the purpose of this project is to design and develop an Unscented Kalman Filter (UKF) that is capable of fusing Laser (LIDAR) and Radar measurements to track a bicycle that travels around a simulated vehicle, offering a much larger accuracy than previous implementations (that is: the Extended Kalman Filter --or EKF). Again, as most of the hardware available for vehicles are small, embedded systems, the UKF has been implemented in C++.

## Rubric points
### Code organization
The code is organized following the standard C++ Object Oriented Programing paradigm (OOP). There are the following classes:
* **Tools**. This class allows (1) the computation of the error of our predictions by means of the Root Mean Squared Error (RMSE), and (2) it also incorporates some useful constants shared through the code.
* **MeasurementPackage**. Another utility class. It encapsulates the raw sensor measurements.
* **GroundTruthPackage**. Similar to the MeasurementPackage class, it stores the ground truth values of the measures used in the EKF.
* **UKF**. The main class: it implements the Unscented Kalman filter and fuses the data from Laser and Radar sensors. 

### Build instructions
The herein provided code has been developed in a Windows 10 platform using [MinGW.](http://www.mingw.org/) The steps for compiling the code are the following, assuming the repository is already cloned and that you are in it with the console (CMD):
1. Create the directory 'build' and enter on it.
2. Make and build the code: `cmake .. -G "MinGW Makefiles" && make`

If correctly done, the file `UnscentedKF.exe` will be placed on the build directory.

### Project dependencies
The dependencies are the following (assuming a Windows OS):
* cmake >= 3.5
  * [Click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
   * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Windows: [MinGW](http://www.mingw.org/)
* Eigen library >= 3.3
  * [Click here for installation instructions](http://eigen.tuxfamily.org/index.php?title=Main_Page)
 
### Project results
A UKF is a two-step estimation problem: (1) a state prediction, in which we predict the state of the object of interest (the bicycle in our case) until the new measurement arrives and (2) the measurement update, in which new observations correct our belief about the state of the object of interest. Recall that for this project we fuse the data from Laser and Radar data, being the first a Cartesian system and the latter a Polar one. 

To test the presented implementation, the file `obj_pose-laser-radar-synthetic-input.txt` is provided in the `Data` folder.

The experiment is launched using the following command (again, assuming that we are in the build folder): 

`UnscentedKF.exe ../data/obj_pose-laser-radar-synthetic-input.txt output.txt`

The RMSE obtained are: **[0.0702523, 0.083293,  0.328501, 0.225654]**, which are bellow the thresholds given in the requisites (that is should be less than or equal to the values [0.09, 0.10, 0.40, 0.30].).

In order to acchieve the goal, the following variables have been tuned using a grid search:
1. `std_a_ = 2.0`
2. `std_yawdd_ = 0.8`
3. `std_radrd_ = 0.32`

## Concluding remarks
In this project, a more accurate tracking system, capable of fusing Radar and Laser measurements while providing robust results has been implemented in C++. The UKF is simpler (at least in when we compare the source code) than its EKF equivalent and also shows a better performance (translated in a lower RMSE), as the experiment reflects. 
