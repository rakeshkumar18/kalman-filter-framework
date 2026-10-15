#include "kf/filters/LinearKalmanFilter.hpp"

kf::LinearKalmanFilter::LinearKalmanFilter(const kf::StateVector &initial_state, 
    const kf::StateMatrix &initial_cov)
    :state_(initial_state), cov_(initial_cov)
    {

    }

const kf::StateVector &kf::LinearKalmanFilter::state() const
{
    return state_;

}

const kf::StateMatrix &kf::LinearKalmanFilter::covariance() const
{
    return cov_;
}

void kf::LinearKalmanFilter::predict(const kf::StateMatrix &F, const kf::StateMatrix &Q){
    state_ = F * state_;
    cov_ = F * cov_ * F.transpose() + Q;
}

void kf::LinearKalmanFilter::update(const kf::MeasurementVector &z,  
    const kf::MeasurementMatrix &H, const kf::MeasurementCovariance &R)
{
    // Predicted Measurement from a priori state
    kf::MeasurementVector z_pred = H * state_;

    // compute Innovation
    kf::MeasurementVector innovation = z - z_pred;

    // compute covatiance matrix

    kf::MeasurementCovariance S = H * cov_ * H.transpose() + R;

    // Calculate Gain

    auto K = cov_ * H.transpose() *  S.inverse()  ;

    // updated step
    state_ += K * innovation;

    // update posterior covariance
    StateMatrix I = StateMatrix::Identity();

    cov_ = (I - K * H) * cov_;
}