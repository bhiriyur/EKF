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
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //check division by zero
    float denom = px*px + py*py;
    std::cout << 'ASSERT BEFORE...';
    assert(denom > 0);
    std::cout << 'DONE' << std::endl;
    float denom2 = sqrt(denom);
    float denom32 = pow(denom, 3/2);

    //compute the Jacobian matrix
    float H11 = px/denom2;
    float H12 = py/denom2;
    float H21 = -py/denom;
    float H22 = px/denom;
    float H31 = py*(vx*py-vy*px)/denom32;
    float H32 = px*(vy*px-vx*py)/denom32;
    float H33 = px/denom2;
    float H34 = py/denom2;

    Hj << H11, H12,   0,   0,
          H21, H22,   0,   0,
          H31, H32, H33, H34;

    return Hj;
}
