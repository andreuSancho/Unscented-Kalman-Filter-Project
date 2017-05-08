#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = Eigen::VectorXd(5);

  // initial covariance matrix
  P_ = Eigen::MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0; //originally set to 30.

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.8; //Originally set to 30.

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.32; //originally set to 0.3

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  // State dimension.
  n_x_ = 5;
  // Augmented state dimension.
  n_aug_ = n_x_ + 2;
  // Avoid repeating the computation.
  n_aug_extra = (unsigned int) (2 * n_aug_ + 1);
  // Sigma point spreading parameter.
  lambda_ = 3.0 - (double)n_aug_;
  // The current NIS for radar.
  NIS_radar_ = 0.0;
  // The current NIS for laser.
  NIS_laser_ = 0.0;
  Xsig_pred_ = Eigen::MatrixXd::Zero(n_x_, n_aug_extra);
  weights_ = Eigen:: VectorXd::Zero(n_aug_extra);
  // Set weights.
  weights_(0) = lambda_/(lambda_ + n_aug_);
  for (unsigned int i = 1; i < n_aug_extra; ++i) {
    weights_(i) = 0.5/(n_aug_ + lambda_);
  }
  n_z_ = 3;
  n_z_laser_ = 2;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
	if(!is_initialized_) {
		P_ = Eigen::MatrixXd(n_x_, n_x_);
		P_ << 1.0, 0.0, 0.0, 0.0, 0.0,
			  0.0, 1.0, 0.0, 0.0, 0.0,
			  0.0, 0.0, 1.0, 0.0, 0.0,
			  0.0, 0.0, 0.0, 1.0, 0.0,
			  0.0, 0.0, 0.0, 0.0, 1.0;
		// Avoid division by zero with the following trick.
		double range = (meas_package.raw_measurements_(0) < Tools::Epsilon)? 
			Tools::Epsilon : meas_package.raw_measurements_(0); 
		double bearing = (meas_package.raw_measurements_(1) < Tools::Epsilon)? 
			Tools::Epsilon : meas_package.raw_measurements_(1); 
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			// Convert radar from polar to cartesian coordinates and initialize state.
			double range_rate = (meas_package.raw_measurements_(2) < Tools::Epsilon)? 
				Tools::Epsilon : meas_package.raw_measurements_(2);
			double x = range * cos(bearing); 
			double y = range * sin(bearing);
			double vx = range_rate * cos(bearing);
			double vy = range_rate * sin(bearing);
			x_ << x, y, sqrt(vx * vx + vy * vy), 0.0, 0.0;
		} else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			x_ << range, bearing, 0.0, 0.0, 0.0; // Initialize the state.
		}  
		is_initialized_ = true;
		previous_timestamp_ = meas_package.timestamp_;
		if(!use_radar_ && !use_laser_) { // Exception: If no sensors set then use radar.
			std::cout << " > Warning [UKF]: Both sensors disabled! Activating radar." << std::endl;
			use_radar_ = true;
		}
		return; //[Andreu]: Not the most elegant way of doing this.
	}
	long long time_difference = (meas_package.timestamp_ - previous_timestamp_);
	previous_timestamp_ = meas_package.timestamp_; // Update time stamp.
	double delta_t = (double)(time_difference / Tools::MICRO2SEC);
	
	// Set the prediction.
	Prediction(delta_t);
	
	// Continue with the update. Now we've to distinguish if the user wants radar or lidar.
	z_prediction_ = Eigen::VectorXd::Zero(n_z_); // Mean predicted measurement.
	S_ = Eigen::MatrixXd::Zero(n_z_, n_z_); // Measurement covariance matrix S.
	if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		PredictRadarMeasurement();
		UpdateRadar(meas_package); 
	} else if(use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
		PredictLidarMeasurement();
		UpdateLidar(meas_package); 
	}
	previous_timestamp_ = meas_package.timestamp_;
}

/**
* It predicts using radar measurements. It uses Xsig_pred_, Zsig_, S_ and z_prediction_ attributes.
*/
void UKF::PredictRadarMeasurement() {
	// Create matrix for sigma points in measurement space.
	Zsig_ = Eigen::MatrixXd(n_z_, n_aug_extra);
	// Transform sigma points into measurement space.
	for (unsigned int i = 0; i < n_aug_extra; ++i) { 
		double p_x = Xsig_pred_(0, i); // Use the member Xsig_pred_ attribute.
		double p_y = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);
		double v1 = cos(yaw) * v;
		double v2 = sin(yaw) * v;
		// Measurement model.
		Zsig_(0, i) = sqrt(p_x * p_x + p_y * p_y);                        		// r
		Zsig_(1, i) = atan2(p_y, p_x);                                 			// phi
		Zsig_(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);   	// r_dot
	}
	//mean predicted measurement.
	Eigen::VectorXd z_pred = Eigen::VectorXd(n_z_);
	z_pred.fill(0.0);
	for (unsigned int i = 0; i < n_aug_extra; ++i) {
		z_pred = z_pred + weights_(i) * Zsig_.col(i);
	}
	//measurement covariance matrix S.
	Eigen::MatrixXd S = Eigen::MatrixXd(n_z_, n_z_);
	S.fill(0.0);
	for (unsigned int i = 0; i < n_aug_extra; ++i) {
		// Residual.
		Eigen::VectorXd z_diff = Zsig_.col(i) - z_pred;
		// Angle normalization.
		while (z_diff(1) > M_PI) { 
			z_diff(1) -= 2.0 * M_PI; 
		}
		while (z_diff(1) < -M_PI) { 
			z_diff(1) += 2.0 * M_PI; 
		}
		S = S + weights_(i) * z_diff * z_diff.transpose();
	}
	// Add measurement noise covariance matrix.
	Eigen::MatrixXd R = Eigen::MatrixXd(n_z_, n_z_);
	R <<  std_radr_ * std_radr_, 0.0, 0.0,
          0.0, std_radphi_ * std_radphi_, 0.0,
          0.0, 0.0, std_radrd_ * std_radrd_;
	S = S + R;
	z_prediction_ = z_pred;
	S_ = S;
}

/**
* It predicts using Laser measurements. It uses Xsig_pred_, Zsig_, S_ and z_prediction_ attributes.
*/
void UKF::PredictLidarMeasurement() {
	// Create matrix for sigma points in measurement space.
	Zsig_ = Eigen::MatrixXd(n_z_laser_, n_aug_extra);
	// Transform sigma points into measurement space.
	for (unsigned int i = 0; i < n_aug_extra; ++i) { 
		// measurement model
		Zsig_(0, i) = Xsig_pred_(0, i);
		Zsig_(1, i) = Xsig_pred_(1, i);
	}
	Eigen::VectorXd z_pred = Eigen::VectorXd(n_z_laser_);
	Eigen::MatrixXd S = Eigen::MatrixXd(n_z_laser_, n_z_laser_);
	z_pred.fill(0.0);
	for (unsigned int i = 0; i < n_aug_extra; ++i) {
		z_pred = z_pred + weights_(i) * Zsig_.col(i);
	}
	S.fill(0.0);
	for (unsigned int i = 0; i < n_aug_extra; ++i) { 
		Eigen::VectorXd z_diff = Zsig_.col(i) - z_pred;
		S = S + weights_(i) * z_diff * z_diff.transpose();
	}
	// Add measurement noise covariance matrix.
	Eigen::MatrixXd R = Eigen::MatrixXd(n_z_laser_, n_z_laser_);
	R <<   std_laspx_ * std_laspx_ , 0.0,
			0.0, std_laspy_ * std_laspy_;
	S = S + R;
	z_prediction_ = z_pred;
	S_ = S;
}

/**
* It generates the sigma points.
* @param Xsig_aug_out is the matrix where the output is placed.
*/
void UKF::GenerateSigmaPoints(Eigen::MatrixXd &Xsig_aug_out) {
	//create augmented mean vector
	Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);
	//create augmented state covariance
	Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_);
	//create sigma point matrix
	Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, n_aug_extra);
	//create augmented mean state.
	x_aug.head(n_x_) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;
	//create augmented covariance matrix.
	P_aug.fill(0.0);
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(5, 5) = std_a_ * std_a_;
	P_aug(6, 6) = std_yawdd_ * std_yawdd_;
	//create square root matrix.
	MatrixXd L = P_aug.llt().matrixL();
	//create augmented sigma points.
	Xsig_aug.col(0) = x_aug;
	for (unsigned int i = 0; i < n_aug_; ++i) {
		Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
		Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
	}
	Xsig_aug_out = Xsig_aug;
}

/**
* It performs the prediction of the sigma points.
* @param Xsig_aug is the augmented matrix.
* @param Xsig_aug_out is the output matrix containing the prediction.
* @param delta_t is the time difference.
*/
void UKF::SigmaPointsPrediction(Eigen::MatrixXd &Xsig_aug, Eigen::MatrixXd &Xsig_aug_out, double delta_t) {
	Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(n_x_, n_aug_extra);
	for (unsigned int i = 0; i < n_aug_extra; ++i) {
		//extract values for better readability
		double p_x = Xsig_aug(0, i);
		double p_y = Xsig_aug(1, i);
		double v = Xsig_aug(2, i);
		double yaw = Xsig_aug(3, i);
		double yawd = Xsig_aug(4, i);
		double nu_a = Xsig_aug(5, i);
		double nu_yawdd = Xsig_aug(6, i);
		// Predicted state values.
		double px_p, py_p;
		// Avoid division by zero.
		if (fabs(yawd) > Tools::Epsilon) {
			px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
		} else {
			px_p = p_x + v * delta_t * cos(yaw);
			py_p = p_y + v * delta_t * sin(yaw);
		}
		double v_p = v;
		double yaw_p = yaw + yawd * delta_t;
		double yawd_p = yawd;
		// Add noise.
		px_p = px_p + 0.5 * nu_a*delta_t * delta_t * cos(yaw);
		py_p = py_p + 0.5 * nu_a*delta_t * delta_t * sin(yaw);
		v_p = v_p + nu_a * delta_t;
		yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
		yawd_p = yawd_p + nu_yawdd * delta_t;
		// write predicted sigma point into right column.
		Xsig_pred(0, i) = px_p;
		Xsig_pred(1, i) = py_p;
		Xsig_pred(2, i) = v_p;
		Xsig_pred(3, i) = yaw_p;
		Xsig_pred(4, i) = yawd_p;
  }
  Xsig_aug_out = Xsig_pred;
}

/**
* It predicts the mean and covariance objects.
* @param Xsig_pred is the prediction matrix.
* @param x is the output matrix where the mean data are stored.
* @param P is the covariance matrix where the covariance data are stored.
*/
void UKF::PredictMeanAndCovariance(Eigen::MatrixXd &Xsig_pred, Eigen::VectorXd &x, Eigen::MatrixXd &P) {
	x.fill(0.0);
	for (unsigned int i = 0; i < n_aug_extra; ++i) {  // Iterate over the sigma points.
		x = x + weights_(i) * Xsig_pred.col(i);
	}
	//predicted state covariance matrix
	P.fill(0.0);
	for (unsigned int i = 0; i < n_aug_extra; ++i) {  // Iterate over the sigma points.
		// State difference.
		Eigen::VectorXd x_diff = Xsig_pred.col(i) - x;
		// Angle normalization.
		while (x_diff(3) >  M_PI) { 
			x_diff(3) -= 2.0 * M_PI; 
		}
		while (x_diff(3) < -M_PI) { 
			x_diff(3) += 2.0 * M_PI; 
		}
		P = P + weights_(i) * x_diff * x_diff.transpose();
	}
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
	Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_extra, n_aug_);
	Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(n_x_, n_aug_extra);
	Eigen::VectorXd x_predicted = Eigen::VectorXd(n_x_);
	Eigen::MatrixXd P_predicted = Eigen::MatrixXd(n_x_, n_x_);
  
	// Generate the sigma points.
	GenerateSigmaPoints(Xsig_aug);
	
	// Sigma points prediction.
	SigmaPointsPrediction(Xsig_aug, Xsig_pred, delta_t);
  
	// Predict mean and covariance.
	PredictMeanAndCovariance(Xsig_pred, x_predicted, P_predicted);
	
	// Update the state.
	x_ = x_predicted;
	P_ = P_predicted;
	Xsig_pred_ = Xsig_pred;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
	Eigen::VectorXd z = Eigen::VectorXd::Zero(n_z_laser_); // Measurements are stored here.
	z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);
	Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n_x_, n_z_laser_); // Cross-correlation matrix Tc.
	// Calculate the cross-correlation matrix.
	Tc.fill(0.0);
	for (unsigned int i = 0; i < n_aug_extra; ++i) { 
		// Residual.
		Eigen::VectorXd z_diff = Zsig_.col(i) - z_prediction_;
		// State difference.
		Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}  
	Eigen::MatrixXd S_inv = S_.inverse();
	// Kalman gain K.
	Eigen::MatrixXd K = Tc * S_inv;
	// Residual.
	Eigen::VectorXd z_diff = z - z_prediction_;
	// Update state mean and covariance matrix.
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S_ * K.transpose();
	NIS_laser_ = z_diff.transpose() * S_inv * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
	Eigen::VectorXd z = Eigen::VectorXd::Zero(n_z_);
	z <<  meas_package.raw_measurements_(0),
			meas_package.raw_measurements_(1),
			meas_package.raw_measurements_(2);
	Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n_x_, n_z_);
	// Calculate the cross-correlation matrix.
	Tc.fill(0.0);
	for (unsigned int i = 0; i < n_aug_extra; ++i) { 
		// Residual.
		Eigen::VectorXd z_diff = Zsig_.col(i) - z_prediction_;
		// Angle normalization.
		while (z_diff(1) >  M_PI) {
			z_diff(1) -= 2.0 * M_PI;
		}
		while (z_diff(1) < -M_PI) {
			z_diff(1) += 2.0 * M_PI;
		}
		// State difference.
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		// Angle normalization.
		while (x_diff(3) >  M_PI) {
			x_diff(3) -= 2.0 * M_PI;
		}
		while (x_diff(3) < -M_PI) {
			x_diff(3) += 2.0 * M_PI;
		}
		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}  
	Eigen::MatrixXd S_inv = S_.inverse();
	// Kalman gain K.
	Eigen::MatrixXd K = Tc * S_inv;
	// Residual.
	Eigen::VectorXd z_diff = z - z_prediction_;
	// Angle normalization
	while (z_diff(1) >  M_PI) {
		z_diff(1) -= 2.0 * M_PI;
	}
	while (z_diff(1) < -M_PI) {
		z_diff(1) += 2.0 * M_PI;
	}
	// Update state mean and covariance matrix.
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S_ * K.transpose();
	NIS_radar_ = z_diff.transpose() * S_inv * z_diff;
}
