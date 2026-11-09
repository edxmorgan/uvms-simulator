#pragma once

// Author: edward morgan

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace uvms_dynamics_test {

constexpr int kUvmsStateDim = 22;
constexpr int kVehicleStateDim = 12;
constexpr int kArmStateDim = 10;
constexpr int kVehicleWrenchDim = 6;
constexpr int kArmTorqueDim = 5;
constexpr int kVehicleParamsDim = 33;
constexpr int kArmParamsDim = 81;
constexpr int kExternalWrenchDim = 6;
constexpr int kLockMaskDim = 4;

inline std::vector<double> default_vehicle_params() {
    return {
        3.72028553e+01, 2.21828075e+01, 6.61734807e+01, 3.389098005e+00,
        6.41362040e-01, 6.41362040e-01, 3.389098005e+00, 1.39646394e+00,
        4.98032205e-01, 2.53118738e+00, 1.05000000e+02, 9.78296453e+01,
        8.27479545e-01, 1.36822559e-01, 4.25841171e+00, -7.36416666e+01,
        -3.36082112e+01, -8.94055107e+01, -2.98736214e+00, -1.57921531e+00,
        -3.39766499e+00, -1.47912104e-04, -5.16373030e-04, -9.85522538e+01,
        -3.05907788e-02, -1.27877517e-01, -1.63514832e+00,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

inline std::vector<double> default_arm_params() {
    return {
        1.94000000e-01, 4.29000000e-01, 1.14999999e-01, 3.32999998e-01,
        -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -4.29000003e-02,
        1.96649101e-02, 4.29000003e-02, 2.88077923e-03, 7.23516749e-03,
        9.16434754e-03, 2.16416476e-03, -1.19076924e-03, 8.07346553e-03,
        7.10109586e-01, 7.10109586e-01, 1.99576149e-06, -0.00000000e+00,
        -0.00000000e+00, -0.00000000e+00, 1.10178508e-01, 1.83331277e-01,
        1.04292121e-01, -3.32240937e-02, -8.30350362e-02, -3.83631263e-02,
        1.18956416e-01, 1.22363853e-01, 4.34411664e-03, -3.96112974e-04,
        -2.13904668e-02, -1.77228242e-03, 1.92510932e-02, 2.56548460e-02,
        7.17220917e-03, 1.48789886e-03, 4.53687373e-04, -1.09861913e-03,
        2.39569756e+00, 2.23596482e+00, 8.19671021e-01, 3.57249665e-01,
        0.0, 0.0, 0.0, 0.0,
        -0.0, -0.0, -0.0, -0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0,
        0.19, 0.0, -0.12, 3.14159, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.04, 0.0, 0.0, 0.0};
}

inline std::vector<double> tile(const std::vector<double>& row, int n) {
    std::vector<double> out;
    out.reserve(row.size() * static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        out.insert(out.end(), row.begin(), row.end());
    }
    return out;
}

inline std::vector<double> make_uvms_state(int batch_size) {
    std::vector<double> x(static_cast<std::size_t>(batch_size) * kUvmsStateDim, 0.0);
    for (int i = 0; i < batch_size; ++i) {
        double* row = x.data() + static_cast<std::size_t>(i) * kUvmsStateDim;
        row[0] = 0.01 * static_cast<double>(i % 5);
        row[1] = -0.005 * static_cast<double>(i % 7);
        row[2] = -0.001 * static_cast<double>(i % 3);
        row[3] = 0.001 * static_cast<double>(i % 4);
        row[4] = -0.001 * static_cast<double>(i % 6);
        row[5] = 0.002 * static_cast<double>(i % 5);
        row[6] = 0.002 * static_cast<double>(i % 3);
        row[7] = -0.001 * static_cast<double>(i % 4);
        row[8] = 0.001 * static_cast<double>(i % 5);
        row[9] = 0.0005 * static_cast<double>(i % 4);
        row[10] = -0.0005 * static_cast<double>(i % 3);
        row[11] = 0.0007 * static_cast<double>(i % 5);
        row[12] = 0.001 * static_cast<double>(i % 3);
        row[13] = 2.1 - 0.001 * static_cast<double>(i % 4);
        row[14] = 0.4 + 0.001 * static_cast<double>(i % 5);
        row[15] = 0.7 - 0.001 * static_cast<double>(i % 6);
        row[16] = 3.1 + 0.001 * static_cast<double>(i % 7);
        for (int j = 0; j < 5; ++j) {
            row[17 + j] = 0.001 * static_cast<double>((i + j) % 5 - 2);
        }
    }
    return x;
}

inline std::vector<double> make_matrix(int batch_size, int cols, double scale) {
    std::vector<double> out(static_cast<std::size_t>(batch_size) * cols, 0.0);
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = scale * static_cast<double>(static_cast<int>(i % 11) - 5);
    }
    return out;
}

inline std::vector<double> filled(int batch_size, int cols, double value) {
    return std::vector<double>(static_cast<std::size_t>(batch_size) * cols, value);
}

inline void split_uvms_state(
    const std::vector<double>& x_uvms,
    int batch_size,
    std::vector<double>* x_vehicle,
    std::vector<double>* x_arm) {
    x_vehicle->assign(static_cast<std::size_t>(batch_size) * kVehicleStateDim, 0.0);
    x_arm->assign(static_cast<std::size_t>(batch_size) * kArmStateDim, 0.0);
    for (int i = 0; i < batch_size; ++i) {
        const double* row = x_uvms.data() + static_cast<std::size_t>(i) * kUvmsStateDim;
        std::copy_n(row, kVehicleStateDim, x_vehicle->data() + static_cast<std::size_t>(i) * kVehicleStateDim);
        std::copy_n(row + kVehicleStateDim, kArmStateDim, x_arm->data() + static_cast<std::size_t>(i) * kArmStateDim);
    }
}

inline std::vector<double> combine_uvms_state(
    const std::vector<double>& x_vehicle,
    const std::vector<double>& x_arm,
    int batch_size) {
    std::vector<double> x_uvms(static_cast<std::size_t>(batch_size) * kUvmsStateDim, 0.0);
    for (int i = 0; i < batch_size; ++i) {
        double* row = x_uvms.data() + static_cast<std::size_t>(i) * kUvmsStateDim;
        std::copy_n(x_vehicle.data() + static_cast<std::size_t>(i) * kVehicleStateDim, kVehicleStateDim, row);
        std::copy_n(x_arm.data() + static_cast<std::size_t>(i) * kArmStateDim, kArmStateDim, row + kVehicleStateDim);
    }
    return x_uvms;
}

inline double max_error(const std::vector<double>& a, const std::vector<double>& b, int* index) {
    if (a.size() != b.size()) {
        throw std::invalid_argument("max_error size mismatch");
    }
    double max_err = 0.0;
    int max_i = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!std::isfinite(a[i]) || !std::isfinite(b[i])) {
            *index = static_cast<int>(i);
            return std::numeric_limits<double>::infinity();
        }
        const double err = std::abs(a[i] - b[i]);
        if (err > max_err) {
            max_err = err;
            max_i = static_cast<int>(i);
        }
    }
    *index = max_i;
    return max_err;
}

}  // namespace uvms_dynamics_test
