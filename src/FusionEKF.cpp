#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

bool DEBUG = false;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225,      0,
                0,      0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09,      0,     0,
                   0, 0.0009,     0,
                   0,      0,  0.09;

    if (DEBUG) {
                cout << "FusionEFK: Finished Initialization" << endl;
        };
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
        TODO:
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */
        // first measurement

        // Initializing the state covariance matrix
        ekf_.P_ = MatrixXd(4, 4);
        ekf_.P_ << 1, 0,    0,    0,
                   0, 1,    0,    0,
                   0, 0, 1000,    0,
                   0, 0,    0, 1000;

        ekf_.x_ = VectorXd(4);

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            float rho0 = measurement_pack.raw_measurements_[0];
            float phi0 = measurement_pack.raw_measurements_[1];
            float v0 =  measurement_pack.raw_measurements_[2];

            float x0 = rho0*cos(phi0);
            float y0 = rho0*sin(phi0);
            float vx0 = v0*cos(phi0);
            float vy0 = v0*sin(phi0);

            ekf_.x_ << x0, y0, vx0, vy0;

        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            //set the state with the initial location and zero velocity
            ekf_.x_ << measurement_pack.raw_measurements_[0],
                       measurement_pack.raw_measurements_[1],
                       1,
                       1;

        }

        // done initializing, no need to predict or update
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
       * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */

    //compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    //Modify the F matrix so that the time is integrated
    ekf_.F_ = MatrixXd::Identity(4, 4);
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    //set the process covariance matrix Q
    float noise_ax = 9;
    float noise_ay = 9;

    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
                0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
                dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
                0, dt_3/2*noise_ay, 0, dt_2*noise_ay;


    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.R_ = R_radar_;
        Tools tools;
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // Laser updates
        ekf_.R_ = R_laser_;
        ekf_.H_ = MatrixXd(2, 4);
        ekf_.H_ <<  1, 0, 0, 0,
                    0, 1, 0, 0;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    if (DEBUG) {
        cout << "x_ = " << ekf_.x_.transpose() << endl;
//        cout << "P_ = " << ekf_.P_ << endl;
    };
}
