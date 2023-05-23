/*
 * This file is part of the EDS: Event-aided Direct Sparse Odometry
 * (https://rpg.ifi.uzh.ch/eds.html)
 *
 * This file is modified and part of the MC-VEO
 * (https://cslinzhang.github.io/MC-VEO/)
 * 
 * Copyright (c) 2022 Javier Hidalgo-Carrió, Robotics and Perception
 * Group (RPG) University of Zurich.
 *
 * MC-VEO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * MC-VEO is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include <map>
#include <vector>
#include <string>

/** Rock base types **/
#include <base/Float.hpp>
#include <base/Eigen.hpp>
#include <base/Point.hpp>
#include <base/samples/Pointcloud.hpp>

#include <mc-veo/utils/NumType.h>
#include <mc-veo/utils/MinimalImage.h>

namespace cv {
        class Mat;
}

namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;

namespace io
{

base::samples::Pointcloud getMap(const dso::FrameHessian *fh, dso::CalibHessian *hcalib,
                                const base::Vector4d color= ::base::Vector4d(::base::NaN<double>(), ::base::NaN<double>(),
                                                            ::base::NaN<double>(), ::base::NaN<double>()), const bool &single_point = true);
base::samples::Pointcloud getImmatureMap(const dso::FrameHessian *fh, dso::CalibHessian *hcalib, const bool &single_point = true);

}
}
