#include <iostream>
#include "tools.h"

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
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;
	VectorXd residual;
	VectorXd mean;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if (estimations.size() == 0)
	{
		cout << "Estimation vector has zero size!";
		return rmse;
	}
	else if (estimations.size() != ground_truth.size())
	{
		cout << "Estimation and ground truth vectors are of varying sizes!";
		return rmse;
	}

	//accumulate squared residuals
	for (int i = 0; i < estimations.size(); ++i) {
		// ... your code here
		residual = estimations[i] - ground_truth[i];
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	// ... your code here
	mean = rmse / estimations.size();

	//calculate the squared root
	// ... your code here
	rmse = mean.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3, 4);

	// get states
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// check division by zero
	if (fabs(((px*px) + (py*py))) < 0.0001)
	{
		cout << "Divide by zero error!";
		return Hj;
	}

	// compute jacobian matrix
	Hj(0, 0) = px / (sqrt((px*px) + (py*py)));
	Hj(0, 1) = py / (sqrt((px*px) + (py*py)));
	Hj(0, 2) = 0;
	Hj(0, 3) = 0;
	Hj(1, 0) = -py / (((px*px) + (py*py)));
	Hj(1, 1) = px / (((px*px) + (py*py)));
	Hj(1, 2) = 0;
	Hj(1, 3) = 0;
	Hj(2, 0) = py*((vx*py) - (vy*px)) / (pow(((px*px) + (py*py)), 1.5));
	Hj(2, 1) = px*((vy*px) - (vx*py)) / (pow(((px*px) + (py*py)), 1.5));
	Hj(2, 2) = Hj(0, 0);
	Hj(2, 3) = Hj(0, 1);
	return Hj;
}
