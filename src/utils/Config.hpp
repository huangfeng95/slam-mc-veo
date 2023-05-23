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


#ifndef _MC-VEO_UTILS_CONFIG_HPP_
#define _MC-VEO_UTILS_CONFIG_HPP_

#include <yaml-cpp/yaml.h>
#include <string>

namespace mc-veo { namespace recorder{

    struct Visual
    {
       bool model;
       bool events;
       bool depth;
       bool flow;
       bool sigma;
       bool weights;
       bool dsi;
       bool map;
    };

    struct Config
    {
        std::string output_folder;
        std::string poses_filename;
        std::string velos_filename;
        Visual viz;
    };

    inline ::mc-veo::recorder::Config readRecorderConfig(YAML::Node config)
    {
        ::mc-veo::recorder::Config recorder_config;

        return recorder_config;
    };

} //recorder namespace
} // end namespace
#endif
