#pragma once
#include <vector>
namespace ros2_control_blue_reach_5 {
// Identified 27-coefficient vehicle model plus six nominal current components.
// Shared by hardware estimation and the default simulated plant; runtime plant
// profile changes must never mutate this nominal estimator model.
inline std::vector<double> default_nominal_vehicle_parameters()
        {
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
}
