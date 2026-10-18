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

void
kf::LinearKalmanFilter::update(
    const kf::MeasurementVector& z,
    const kf::MeasurementMatrix& H,
    const kf::MeasurementCovariance& R
)
{
    // ----------------------------------------
    // Predicted measurement
    // ----------------------------------------

    kf::MeasurementVector z_pred =
        H * state_;


    // ----------------------------------------
    // Innovation
    // y = z - Hx-
    // ----------------------------------------

    innovation_ =
        z - z_pred;


    // ----------------------------------------
    // Innovation covariance
    // S = H P- H^T + R
    // ----------------------------------------

    kf::MeasurementCovariance S =
        H
        * cov_
        * H.transpose()
        + R;


    // ----------------------------------------
    // Kalman gain
    // K = P- H^T S^-1
    // ----------------------------------------

    kalman_gain_ =
        cov_
        * H.transpose()
        * S.inverse();


    // ----------------------------------------
    // State correction
    // x+ = x- + Ky
    // ----------------------------------------

    state_ =
        state_
        +
        kalman_gain_
        * innovation_;


    // ----------------------------------------
    // Covariance correction
    // P+ = (I - KH)P-
    // ----------------------------------------

    kf::StateMatrix I =
        kf::StateMatrix::Identity();


    cov_ =
        (
            I
            -
            kalman_gain_
            * H
        )
        * cov_;
}

const kf::MeasurementVector& kf::LinearKalmanFilter::innovation() const
{
    return innovation_;
}


const kf::KalmanGainMatrix& kf::LinearKalmanFilter::kalmanGain() const
{
    return kalman_gain_;
}