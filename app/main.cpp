#include <iostream>
#include <iomanip>
#include <vector>

#include "kf/simulation/VehicleSimulator.hpp"
#include "kf/sensors/GPSSensor.hpp"
#include "kf/models/ConstantVelocityModel.hpp"
#include "kf/models/GPSMeasurementModel.hpp"
#include "kf/filters/LinearKalmanFilter.hpp"
#include "kf/core/EstimatorSnapshot.hpp"
#include "kf/visualization/RealTimeViewer.hpp"

int main()
{
    // ------------------------------------------------
    // 1. Initial true state
    // ------------------------------------------------
    kf::StateVector initial_state;

    initial_state << 0.0,
                     0.0,
                     5.0,
                     1.0;

    kf::VehicleSimulator simulator(initial_state);


    // ------------------------------------------------
    // 2. GPS sensor
    // ------------------------------------------------
    kf::GPSSensor gps(
        1.0,    // sigma_x
        1.5     // sigma_y
    );


    // ------------------------------------------------
    // 3. Motion + measurement models
    // ------------------------------------------------
    kf::ConstantVelocityModel motion_model;
    kf::GPSMeasurementModel gps_model;

    kf::MeasurementMatrix H =
        gps_model.measurementMatrix();


    // ------------------------------------------------
    // 4. Initial filter
    // ------------------------------------------------
    kf::StateVector initial_estimate = initial_state;

    kf::StateMatrix initial_cov =
        kf::StateMatrix::Identity();

    kf::LinearKalmanFilter filter(
        initial_estimate,
        initial_cov
    );


    // ------------------------------------------------
    // 5. Simulation parameters
    // ------------------------------------------------
    kf::Scalar dt = 0.1;

    kf::Scalar ax = 0.2;
    kf::Scalar ay = 0.0;


    // Process acceleration covariance
    kf::AccelerationCovariance acc_cov =
        kf::AccelerationCovariance::Zero();

    acc_cov(0, 0) = 0.2;
    acc_cov(1, 1) = 0.05;


    // GPS measurement covariance
    kf::MeasurementCovariance R(2, 2);

    R << 1.0, 0.0,
         0.0, 2.25;


    std::cout << std::fixed
              << std::setprecision(3);

    // std::cout
    //     << "time\t"
    //     << "true_x\ttrue_y\ttrue_vx\ttrue_vy\t"
    //     << "gps_x\tgps_y\t"
    //     << "pred_x\tpred_y\tpred_vx\tpred_vy\t"
    //     << "est_x\test_y\test_vx\test_vy\n";


    // ------------------------------------------------
    // 6. Simulation loop
    // ------------------------------------------------
    // std::vector<kf::EstimatorSnapshot> history;

    // history.reserve(100);
    // for (int step = 0; step < 10; ++step)
    // {
    //     // Ground truth
    //     simulator.step(dt, ax, ay);

    //     const kf::StateVector& true_state =
    //         simulator.state();


    //     // GPS measurement
    //     kf::MeasurementVector z =
    //         gps.measure(true_state);


    //     // Filter model
    //     kf::StateMatrix F =
    //         motion_model.stateTransition(dt);

    //     kf::StateMatrix Q =
    //         motion_model.processNoise(
    //             dt,
    //             acc_cov
    //         );


    //     // Prediction
    //     filter.predict(F, Q);


    //     // SAVE PRIOR before update
    //     kf::StateVector predicted_state =
    //         filter.state();

    //     kf::StateMatrix predicted_cov =
    //         filter.covariance();


    //     // Measurement update
    //     filter.update(z, H, R);

    //     kf::Scalar sim_time =
    //         (step + 1) * dt;

    //     kf::EstimatorSnapshot snapshot;

    //     snapshot.time = sim_time;

    //     snapshot.true_state = true_state;

    //     snapshot.gps_measurement = z;

    //     snapshot.predicted_state =
    //         predicted_state;

    //     snapshot.predicted_covariance =
    //         predicted_cov;

    //     snapshot.estimated_state =
    //         filter.state();

    //     snapshot.estimated_covariance =
    //         filter.covariance();

    //     const kf::StateVector& estimated_state =
    //         filter.state();
        
    //     history.push_back(snapshot);

    //     std::cout
    //         << sim_time << '\t'

    //         << true_state(0) << '\t'
    //         << true_state(1) << '\t'
    //         << true_state(2) << '\t'
    //         << true_state(3) << '\t'

    //         << z(0) << '\t'
    //         << z(1) << '\t'

    //         << predicted_state(0) << '\t'
    //         << predicted_state(1) << '\t'
    //         << predicted_state(2) << '\t'
    //         << predicted_state(3) << '\t'

    //         << estimated_state(0) << '\t'
    //         << estimated_state(1) << '\t'
    //         << estimated_state(2) << '\t'
    //         << estimated_state(3)
    //         << '\n';
    // }

    std::vector<kf::EstimatorSnapshot> history;

    history.reserve(100);
    int step = 0;

    const int max_steps = 100;

    auto step_callback =
    [&](kf::EstimatorSnapshot& snapshot) -> bool
    {
        // Stop simulation after max_steps
        if (step >= max_steps)
        {
            return false;
        }


        // --------------------------------------------
        // 1. Ground truth
        // --------------------------------------------

        simulator.step(
            dt,
            ax,
            ay
        );

        const kf::StateVector& true_state =
            simulator.state();


        // --------------------------------------------
        // 2. GPS measurement
        // --------------------------------------------

        kf::MeasurementVector z =
            gps.measure(true_state);


        // --------------------------------------------
        // 3. Model matrices
        // --------------------------------------------

        kf::StateMatrix F =
            motion_model.stateTransition(dt);

        kf::StateMatrix Q =
            motion_model.processNoise(
                dt,
                acc_cov
            );


        // --------------------------------------------
        // 4. Kalman prediction
        // --------------------------------------------

        filter.predict(
            F,
            Q
        );


        // Preserve x- and P-
        kf::StateVector predicted_state =
            filter.state();

        kf::StateMatrix predicted_cov =
            filter.covariance();


        // --------------------------------------------
        // 5. Measurement correction
        // --------------------------------------------

        filter.update(
            z,
            H,
            R
        );


        // --------------------------------------------
        // 6. Simulation timestamp
        // --------------------------------------------

        kf::Scalar sim_time =
            (step + 1) * dt;


        // --------------------------------------------
        // 7. Fill snapshot
        // --------------------------------------------

        snapshot.time = sim_time;

        snapshot.true_state = true_state;

        snapshot.gps_measurement = z;

        snapshot.predicted_state = predicted_state;

        snapshot.predicted_covariance = predicted_cov;

        snapshot.estimated_state = filter.state();

        snapshot.estimated_covariance = filter.covariance();

        snapshot.innovation = filter.innovation();

        snapshot.kalman_gain = filter.kalmanGain();
        ++step;

        return true;
    };

    // std::cout << "\nHistory size: "
    //       << history.size()
    //       << '\n';
    
    // const kf::EstimatorSnapshot& last =
    // history.back();

    // std::cout << "\nLast snapshot\n";

    // std::cout
    //     << "time = "
    //     << last.time
    //     << '\n';

    // std::cout
    //     << "true state:\n"
    //     << last.true_state
    //     << "\n\n";

    // std::cout
    //     << "predicted state:\n"
    //     << last.predicted_state
    //     << "\n\n";

    // std::cout
    //     << "estimated state:\n"
    //     << last.estimated_state
    //     << '\n';


    // kf::RealTimeViewer viewer;

    // if (!viewer.initialize())
    // {
    //     std::cerr << "Failed to initialize viewer\n";
    //     return 1;
    // }

    // viewer.run(history);

    kf::RealTimeViewer viewer;

    if (!viewer.initialize())
    {
        std::cerr
            << "Failed to initialize viewer\n";

        return 1;
    }

    viewer.runLive(
        history,
        step_callback,
        dt,
        0.2,     // Qx
        0.05,    // Qy
        1.0,     // Rx
        2.25     // Ry
    );

    return 0;
}