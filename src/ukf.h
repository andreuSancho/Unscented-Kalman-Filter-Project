#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  ///* state covariance matrix
  Eigen::MatrixXd P_;

  ///* predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  Eigen::VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;
  
  // To avoid repeating the same computation (2 * n_aug_ + 1) again and again.
  unsigned int n_aug_extra;
  
  // Previous timestamp.
  long long previous_timestamp_;
  
  // Measurement dimension, radar can measure r, phi, and r_dot.
  int n_z_;
  
  // Measurement dimension, laser can measure x and y.
  int n_z_laser_;
  
  // Sigma points in measurement space.
  Eigen::MatrixXd Zsig_;
  
  // Mean predicted measurement.
  Eigen::VectorXd z_prediction_;
  
  // Measurement covariance matrix-
  Eigen::MatrixXd S_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
  
	/**
	* It generates the sigma points.
	* @param Xsig_aug_out is the matrix where the output is placed.
	*/
	void GenerateSigmaPoints(Eigen::MatrixXd &Xsig_aug_out);
	
	/**
	* It performs the prediction of the sigma points.
	* @param Xsig_aug is the augmented matrix.
	* @param Xsig_aug_out is the output matrix containing the prediction.
	* @param delta_t is the time difference.
	*/
	void SigmaPointsPrediction(Eigen::MatrixXd &Xsig_aug, Eigen::MatrixXd &Xsig_aug_out, double delta_t);
	
	/**
	* It predicts the mean and covariance objects.
	* @param Xsig_pred is the prediction matrix.
	* @param x is the output matrix where the mean data are stored.
	* @param P is the covariance matrix where the covariance data are stored.
	*/
	void PredictMeanAndCovariance(Eigen::MatrixXd &Xsig_pred, Eigen::VectorXd &x, Eigen::MatrixXd &P);
	
	/**
	* It predicts using radar measurements.  It uses Xsig_pred_, Zsig_, S_ and z_prediction_ attributes.
	*/
	void PredictRadarMeasurement();
	
	/**
	* It predicts using Laser measurements. It uses Xsig_pred_, Zsig_, S_ and z_prediction_ attributes.
	*/
	void PredictLidarMeasurement();
};

#endif /* UKF_H */
