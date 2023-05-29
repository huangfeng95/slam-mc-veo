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
#include <vector>
#include <base/Time.hpp>
#include <base/samples/Frame.hpp>
#include <mc-veo/utils/NumType.h>
#include <mc-veo/utils/MinimalImage.h>


namespace mc_veo
{

namespace io
{

dso::MinimalImageB3* toMinimalImageB3(const Eigen::Vector3f *fd, const int &w, const int &h);
void MinimalImageB3ToFrame(const dso::MinimalImageB3 *input, const ::base::Time &timestamp, ::base::samples::frame::Frame &frame);

}
}
