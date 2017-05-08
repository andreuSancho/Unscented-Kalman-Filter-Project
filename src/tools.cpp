#include <iostream>
#include "tools.h"


double const Tools::Epsilon = 0.0001;
double const Tools::MICRO2SEC = 1000000.0;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  Eigen::VectorXd rmse(4);
	rmse << 0.0, 0.0, 0.0, 0.0;
    if(estimations.size() <= 0) {
        std::cout << " > Error [RMSE]: Estimation has size zero!" << std::endl;
        return(rmse);
    }
    if(estimations.size() != ground_truth.size()) {
        std::cout << " > Error [RMSE]: Estimation and ground truth missmatch sizes!" << std::endl;
        return(rmse);
    }
	// Accumulate squared residuals.
	for(unsigned int i = 0; i < estimations.size(); ++i){
		Eigen::VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array() * residual.array();
		rmse += residual;
  }

  // Calculate the mean.
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}
