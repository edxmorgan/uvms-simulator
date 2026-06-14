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

inline std::vector<float> default_vehicle_params() {
    return {
        3.72028553e+01F, 2.21828075e+01F, 6.61734807e+01F, 3.38909801e+00F,
        6.41362046e-01F, 6.41362034e-01F, 3.38909800e+00F, 1.39646394e+00F,
        4.98032205e-01F, 2.53118738e+00F, 1.05000000e+02F, 9.78296453e+01F,
        8.27479545e-01F, 1.36822559e-01F, 4.25841171e+00F, -7.36416666e+01F,
        -3.36082112e+01F, -8.94055107e+01F, -2.98736214e+00F, -1.57921531e+00F,
        -3.39766499e+00F, -1.47912104e-04F, -5.16373030e-04F, -9.85522538e+01F,
        -3.05907788e-02F, -1.27877517e-01F, -1.63514832e+00F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
}

inline std::vector<float> default_arm_params() {
    return {
        1.94000000e-01F, 4.29000000e-01F, 1.14999999e-01F, 3.32999998e-01F,
        -0.00000000e+00F, -0.00000000e+00F, -0.00000000e+00F, -4.29000003e-02F,
        1.96649101e-02F, 4.29000003e-02F, 2.88077923e-03F, 7.23516749e-03F,
        9.16434754e-03F, 2.16416476e-03F, -1.19076924e-03F, 8.07346553e-03F,
        7.10109586e-01F, 7.10109586e-01F, 1.99576149e-06F, -0.00000000e+00F,
        -0.00000000e+00F, -0.00000000e+00F, 1.10178508e-01F, 1.83331277e-01F,
        1.04292121e-01F, -3.32240937e-02F, -8.30350362e-02F, -3.83631263e-02F,
        1.18956416e-01F, 1.22363853e-01F, 4.34411664e-03F, -3.96112974e-04F,
        -2.13904668e-02F, -1.77228242e-03F, 1.92510932e-02F, 2.56548460e-02F,
        7.17220917e-03F, 1.48789886e-03F, 4.53687373e-04F, -1.09861913e-03F,
        2.39569756e+00F, 2.23596482e+00F, 8.19671021e-01F, 3.57249665e-01F,
        0.0F, 0.0F, 0.0F, 0.0F,
        -0.0F, -0.0F, -0.0F, -0.0F,
        0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F,
        0.0F,
        0.19F, 0.0F, -0.12F, 3.14159F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.04F, 0.0F, 0.0F, 0.0F};
}

inline std::vector<float> tile(const std::vector<float>& row, int n) {
    std::vector<float> out;
    out.reserve(row.size() * static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        out.insert(out.end(), row.begin(), row.end());
    }
    return out;
}

inline std::vector<float> make_uvms_state(int batch_size) {
    std::vector<float> x(static_cast<std::size_t>(batch_size) * kUvmsStateDim, 0.0F);
    for (int i = 0; i < batch_size; ++i) {
        float* row = x.data() + static_cast<std::size_t>(i) * kUvmsStateDim;
        row[0] = 0.01F * static_cast<float>(i % 5);
        row[1] = -0.005F * static_cast<float>(i % 7);
        row[2] = -0.001F * static_cast<float>(i % 3);
        row[3] = 0.001F * static_cast<float>(i % 4);
        row[4] = -0.001F * static_cast<float>(i % 6);
        row[5] = 0.002F * static_cast<float>(i % 5);
        row[6] = 0.002F * static_cast<float>(i % 3);
        row[7] = -0.001F * static_cast<float>(i % 4);
        row[8] = 0.001F * static_cast<float>(i % 5);
        row[9] = 0.0005F * static_cast<float>(i % 4);
        row[10] = -0.0005F * static_cast<float>(i % 3);
        row[11] = 0.0007F * static_cast<float>(i % 5);
        row[12] = 0.001F * static_cast<float>(i % 3);
        row[13] = 2.1F - 0.001F * static_cast<float>(i % 4);
        row[14] = 0.4F + 0.001F * static_cast<float>(i % 5);
        row[15] = 0.7F - 0.001F * static_cast<float>(i % 6);
        row[16] = 3.1F + 0.001F * static_cast<float>(i % 7);
        for (int j = 0; j < 5; ++j) {
            row[17 + j] = 0.001F * static_cast<float>((i + j) % 5 - 2);
        }
    }
    return x;
}

inline std::vector<float> make_matrix(int batch_size, int cols, float scale) {
    std::vector<float> out(static_cast<std::size_t>(batch_size) * cols, 0.0F);
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = scale * static_cast<float>(static_cast<int>(i % 11) - 5);
    }
    return out;
}

inline std::vector<float> filled(int batch_size, int cols, float value) {
    return std::vector<float>(static_cast<std::size_t>(batch_size) * cols, value);
}

inline void split_uvms_state(
    const std::vector<float>& x_uvms,
    int batch_size,
    std::vector<float>* x_vehicle,
    std::vector<float>* x_arm) {
    x_vehicle->assign(static_cast<std::size_t>(batch_size) * kVehicleStateDim, 0.0F);
    x_arm->assign(static_cast<std::size_t>(batch_size) * kArmStateDim, 0.0F);
    for (int i = 0; i < batch_size; ++i) {
        const float* row = x_uvms.data() + static_cast<std::size_t>(i) * kUvmsStateDim;
        std::copy_n(row, kVehicleStateDim, x_vehicle->data() + static_cast<std::size_t>(i) * kVehicleStateDim);
        std::copy_n(row + kVehicleStateDim, kArmStateDim, x_arm->data() + static_cast<std::size_t>(i) * kArmStateDim);
    }
}

inline std::vector<float> combine_uvms_state(
    const std::vector<float>& x_vehicle,
    const std::vector<float>& x_arm,
    int batch_size) {
    std::vector<float> x_uvms(static_cast<std::size_t>(batch_size) * kUvmsStateDim, 0.0F);
    for (int i = 0; i < batch_size; ++i) {
        float* row = x_uvms.data() + static_cast<std::size_t>(i) * kUvmsStateDim;
        std::copy_n(x_vehicle.data() + static_cast<std::size_t>(i) * kVehicleStateDim, kVehicleStateDim, row);
        std::copy_n(x_arm.data() + static_cast<std::size_t>(i) * kArmStateDim, kArmStateDim, row + kVehicleStateDim);
    }
    return x_uvms;
}

inline float max_error(const std::vector<float>& a, const std::vector<float>& b, int* index) {
    if (a.size() != b.size()) {
        throw std::invalid_argument("max_error size mismatch");
    }
    float max_err = 0.0F;
    int max_i = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!std::isfinite(a[i]) || !std::isfinite(b[i])) {
            *index = static_cast<int>(i);
            return std::numeric_limits<float>::infinity();
        }
        const float err = std::abs(a[i] - b[i]);
        if (err > max_err) {
            max_err = err;
            max_i = static_cast<int>(i);
        }
    }
    *index = max_i;
    return max_err;
}

}  // namespace uvms_dynamics_test
