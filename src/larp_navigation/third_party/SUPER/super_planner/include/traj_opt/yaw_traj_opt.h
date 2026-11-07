/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <iostream>
#include <vector>
#include <functional>

#include "utils/geometry/geometry_utils.h"
#include "traj_opt/config.hpp"
#include <utils/optimization/minco.h>

#include <utils/header/type_utils.hpp>
#include <super_utils/scope_timer.hpp>

namespace traj_opt {
    using namespace geometry_utils;
    using namespace optimization_utils;
    using std::cout;
    using std::endl;
    using std::string;
    using std::vector;

    class YawTrajOpt {
    private:
        std::function<double(const Vec3f&,double)> heading_selector_;
        bool free_goal_{false};
        double yaw_dot_max_{10};
        double yaw_acc_max_{2};

    public:

        explicit YawTrajOpt(const double &_yaw_dot_max, double yaw_acc_max=2.0);

        typedef std::shared_ptr<YawTrajOpt> Ptr;
        void setHeadingSelector(std::function<double(const Vec3f&,double)> selector) {heading_selector_=std::move(selector);}

        void getYawTimeAllocation(const double &duration, VecDf &times) const ;

        void getYawWaypointAllocation(const Vec4f &init_state, Vec4f &goal_state, VecDf &way_pts, VecDf &times,
                                      const Trajectory &pos_traj) ;

        bool optimize(const Vec4f &istate_in,
                      const Vec4f &gstate_in,
                      const Trajectory &pos_traj,
                      Trajectory &out_traj,
                      const int & order = 3,
                      const bool &free_start = false,
                      const bool &free_goal = true);

    };


}