#include <iostream>
#include <Eigen/Dense>
#include "kf/core/Types.hpp"
#include "kf/models/ConstantVelocityModel.hpp"
#include "kf/filters/LinearKalmanFilter.hpp"
#include "kf/models/GPSMeasurementModel.hpp"

int main()
{   
    
    
    // Prediction state
    kf::StateVector x0;
    kf::StateMatrix P0;

    x0 << 0.0,
          0.0,
          5.0,
          1.0;

    P0 = kf::StateMatrix::Identity();

    // create filter :
    kf::LinearKalmanFilter filter(x0, P0);

    kf::ConstantVelocityModel model;
    kf::Scalar dt{0.1};

    kf::StateMatrix F = model.stateTransition(dt);

    std::cout<<"State transition Matrix F: " << std::endl << F << std::endl;

    kf::AccelerationCovariance acc_cov;
    acc_cov << 0.5, 0 , 0, 1.2;
    kf::StateMatrix Q = model.processNoise(dt, acc_cov);
    std::cout<<"Procee Noise Matrix Q: " << std::endl << Q << std::endl;

    std::cout << "State before prediction:\n";
    std::cout << filter.state() << "\n\n";


    filter.predict(F, Q);

    std::cout << "State After prediction:\n";
    std::cout << filter.state() << "\n\n";

    std::cout << "Covariance after prediction:\n";
    std::cout << filter.covariance() << '\n';

    kf::GPSMeasurementModel::MeasurementCovariance R = kf::GPSMeasurementModel::MeasurementCovariance::Zero();
    R(0, 0) = 2.0;   // variance in GPS x
    R(1, 1) = 3.0;   // variance in GPS y

    kf::GPSMeasurementModel gps_model;
    auto H = gps_model.measurementMatrix();

    std::cout << "GPS measurement matrix H:\n";
    std::cout << H << "\n\n";

    std::cout << "GPS covariance R:\n";
    std::cout << R << '\n';

    kf::GPSMeasurementModel::MeasurementVector z;

    z << 0.6,
        0.0;

    filter.update(z, H, R);
    std::cout << "updated state\n " << filter.state() << std::endl;
    std::cout << "updated covariance\n " << filter.covariance() << std::endl;

    return 0;
}