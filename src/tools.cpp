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
    * Calculate the RMSE here.
  */

    // Input validation
    assert(estimations.size()>0);
    assert(estimations.size()==ground_truth.size());

    VectorXd rmse(4);
    rmse << 0,0,0,0;

    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        VectorXd diff(4);
        diff = (estimations[i]-ground_truth[i]).array().pow(2);
        rmse += diff;
        float check = rmse(1) / estimations.size();
        // std::cout << sqrt(check) << std::endl;
    }

    //calculate the mean
    rmse /= estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);

    //recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    //check division by zero
    double denom = px*px + py*py;
    if (denom < 0.0001) {
        std::cout << "DENOM-0" << std::endl;
        Hj <<   1, 1, 0, 0,
                1, 1, 0, 0,
                1, 1, 1, 1;
        return Hj;
    }
    double denom2 = std::sqrt(denom);
    double denom32 = std::pow(denom, 1.5);

    //compute the Jacobian matrix
    double H11 = px/denom2;
    double H12 = py/denom2;
    double H21 = -py/denom;
    double H22 = px/denom;
    double H31 = py*(vx*py-vy*px)/denom32;
    double H32 = px*(vy*px-vx*py)/denom32;
    double H33 = px/denom2;
    double H34 = py/denom2;

    Hj << H11, H12,   0,   0,
          H21, H22,   0,   0,
          H31, H32, H33, H34;

    return Hj;
}
