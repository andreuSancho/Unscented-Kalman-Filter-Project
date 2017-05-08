#ifndef TOOLS_H_
#define TOOLS_H_
#include <iostream>
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
	// Value used for detecting/avoiding division by zero.
	static double const Epsilon; 
	// Another useful constant.
	static double const MICRO2SEC; 

  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

};

#endif /* TOOLS_H_ */
