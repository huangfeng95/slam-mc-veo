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

#ifndef _MC_VEO_BUNDLES_CONFIG_HPP_
#define _MC_VEO_BUNDLES_CONFIG_HPP_

#include <base/Time.hpp>

#include <vector>
#include <string>

namespace mc_veo { namespace bundles{

    enum LOSS_FUNCTION{NONE, HUBER, CAUCHY};
    enum LINEAR_SOLVER_TYPE{DENSE_QR, DENSE_SCHUR, SPARSE_SCHUR, SPARSE_NORMAL_CHOLESKY};

    struct SolverOptions
    {
        LINEAR_SOLVER_TYPE linear_solver_type;
        int num_threads;
        int max_num_iterations;
        double function_tolerance;
        bool minimizer_progress_to_stdout;
    };

    struct Config
    {
        std::string type;
        double percent_points;
        double percent_marginalize_vis;
        uint16_t window_size;
        LOSS_FUNCTION loss_type;
        std::vector<double> loss_params;
        SolverOptions options;
    };

    struct PBAInfo
    {
        base::Time time;
        double meas_time_ms;
        int num_iterations;
        double time_seconds;
        uint8_t success;
    };

    inline ::mc_veo::bundles::LOSS_FUNCTION selectLoss(const std::string &loss_name)
    {
        if (loss_name.compare("Huber") == 0)
            return mc_veo::bundles::HUBER;
        else if (loss_name.compare("Cauchy") == 0)
            return mc_veo::bundles::CAUCHY;
        else
            return mc_veo::bundles::NONE;
    };

    inline ::mc_veo::bundles::LINEAR_SOLVER_TYPE selectSolver(const std::string &solver_name)
    {
        if (solver_name.compare("DENSE_QR") == 0)
            return mc_veo::bundles::DENSE_QR;
        else if (solver_name.compare("DENSE_SCHUR") == 0)
            return mc_veo::bundles::DENSE_SCHUR;
        else if (solver_name.compare("SPARSE_SCHUR") == 0)
            return mc_veo::bundles::SPARSE_SCHUR;
        else
            return mc_veo::bundles::SPARSE_NORMAL_CHOLESKY;
    };

    inline ::mc_veo::bundles::Config readBundlesConfig(YAML::Node config)
    {
        ::mc_veo::bundles::Config bundles_config;

        /** Number of points to optimize within the current window **/
        bundles_config.percent_points = config["percent_points"].as<double>();
        /** Percent of visual point to seletc the kf to marginalize **/
        bundles_config.percent_marginalize_vis = config["percent_marginalize_vis"].as<double>();
        /** BA Windows size **/
        bundles_config.window_size = config["window_size"].as<uint16_t>();
        /** Config for tracker type (only ceres) **/
        bundles_config.type = config["type"].as<std::string>();

        /** Config the loss **/
        YAML::Node bundles_loss = config["loss_function"];
        std::string loss_name = bundles_loss["type"].as<std::string>();
        bundles_config.loss_type = ::mc_veo::bundles::selectLoss(loss_name);
        bundles_config.loss_params = bundles_loss["param"].as< std::vector<double> >();

        /** Config for ceres options **/
        YAML::Node tracker_options = config["options"];
        bundles_config.options.linear_solver_type = ::mc_veo::bundles::selectSolver(tracker_options["solver_type"].as<std::string>());
        bundles_config.options.num_threads = tracker_options["num_threads"].as<int>();
        bundles_config.options.max_num_iterations = tracker_options["max_num_iterations"].as<int>();
        bundles_config.options.function_tolerance = tracker_options["function_tolerance"].as<double>();
        bundles_config.options.minimizer_progress_to_stdout = tracker_options["minimizer_progress_to_stdout"].as<bool>();

        return bundles_config;
    };

} //bundles namespace
} // end namespace

#endif
