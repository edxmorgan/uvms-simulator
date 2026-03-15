// Copyright (C) 2024 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.


#include "ros2_control_blue_reach_5/utils.hpp"

void casadi_reach_alpha_5::Utils::usage_cplusplus_checks(const std::string &name, const std::string &bin_name, const std::string &node_name)
{
    std::cout << "---" << node_name << "---" << std::endl;
    std::cout << "Usage from CasADi C++:" << std::endl;
    std::cout << std::endl;

    // Use CasADi's "external" to load a test compiled function
    Function f = external(name, bin_name);

    // Use like any other CasADi function
    std::vector<double> x = {1, 2, 3, 4};
    std::vector<DM> arg = {reshape(DM(x), 2, 2), 5};
    std::vector<DM> res = f(arg);

    std::cout << "result (0): " << res.at(0) << std::endl;
    std::cout << "result (1): " << res.at(1) << std::endl;
}

Function casadi_reach_alpha_5::Utils::load_casadi_fun(const std::string &name, const std::string &bin_name)
{
    // Use CasADi's "external" to load compiled function
    return external(name, bin_name);
}