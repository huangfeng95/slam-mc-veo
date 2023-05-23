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

#ifndef _MC-VEO_TRACKER_HPP_
#define _MC-VEO_TRACKER_HPP_

#include <mc-veo/tracking/Types.hpp>
#include <mc-veo/tracking/Config.hpp>
#include <mc-veo/tracking/KeyFrame.hpp>
#include <mc-veo/tracking/EventFrame.hpp>
#include <memory>
#include <vector>
#include <chrono>

namespace mc-veo { namespace tracking{

enum LOSS_PARAM_METHOD{CONSTANT, MAD, STD};

class Tracker
{
    public:
        /** Configuration **/
        ::mc-veo::tracking::Config config;

    private:
        /** Pointer to KeyFrame **/
        std::shared_ptr<mc-veo::tracking::KeyFrame> kf;

        /** Optimization parameters **/
        Eigen::Vector3d px;
        Eigen::Quaterniond qx;
        Eigen::Matrix<double, 6, 1> vx;

        /** Status Information **/
        mc-veo::tracking::TrackerInfo info;

        double op_time = 0.0;
        int op_ef_num = 0;

        /** Vector of the last N poses **/
        std::vector<mc-veo::SE3> poses;

        /** Squared Norm Mean Flow **/
        double squared_norm_flow;

    public:
        /** @brief Default constructor */
        Tracker(std::shared_ptr<mc-veo::tracking::KeyFrame> kf, const mc-veo::tracking::Config &config);
        
        /** @brief Default constructor */
        Tracker(const mc-veo::tracking::Config &config);

        void reset(std::shared_ptr<mc-veo::tracking::KeyFrame> kf, const Eigen::Vector3d &px, const Eigen::Quaterniond &qx, const bool &keep_velo = true);

        void reset(std::shared_ptr<mc-veo::tracking::KeyFrame> kf, const Eigen::Vector3d &px, const Eigen::Quaterniond &qx, const base::Vector6d &velo);

        void set(const base::Transform3d &T_kf_ef);

        void optimize(const int &id, const std::vector<double> *event_frame, ::base::Transform3d &T_kf_ef,
                    const Eigen::Vector3d &px, const Eigen::Quaterniond &qx, 
                    const mc-veo::tracking::LOSS_PARAM_METHOD loss_param_method);

        void optimize(const int &id, const std::vector<double> *event_frame, ::base::Transform3d &T_kf_ef,
                    const Eigen::Matrix<double, 6, 1> &vx, const mc-veo::tracking::LOSS_PARAM_METHOD loss_param_method);

        bool optimize(const int &id, const std::vector<double> *event_frame, ::base::Transform3d &T_kf_ef,
                    const mc-veo::tracking::LOSS_PARAM_METHOD loss_param_method = mc-veo::tracking::LOSS_PARAM_METHOD::MAD);

        bool optimize(const int &id, const double event_time, const Eigen::Affine3d &pose_w_kf, const std::vector<double> *event_frame, ::base::Transform3d &T_kf_ef,
                    const mc-veo::tracking::LOSS_PARAM_METHOD loss_param_method = mc-veo::tracking::LOSS_PARAM_METHOD::MAD);
                    
        ::base::Transform3d getTransform();

        ::base::Transform3d getTransform(bool &result);

        Eigen::Matrix<double, 6, 1>& getVelocity();

        const Eigen::Vector3d linearVelocity();

        const Eigen::Vector3d angularVelocity();

        std::vector<double> getLossParams(mc-veo::tracking::LOSS_PARAM_METHOD method=CONSTANT);

        /** Get warpped active points coordinates (point in event frame) **/
        std::vector<cv::Point2d> getCoord(const bool &delete_out_point = false);

        void trackPoints(const cv::Mat &event_frame, const uint16_t &patch_radius = 7);

        void trackPointsPyr(const cv::Mat &event_frame, const size_t num_level = 3);

        std::vector<cv::Point2d> trackPointsAlongEpiline(const cv::Mat &event_frame, const uint16_t &patch_radius = 7,
                            const int &border_type = cv::BORDER_DEFAULT, const uint8_t &border_value = 255);

        cv::Mat getEMatrix();

        cv::Mat getFMatrix();

        ::mc-veo::tracking::TrackerInfo getInfo();

        bool getFilteredPose(mc-veo::SE3 &pose, const size_t &mean_filter_size = 3);

        bool needNewKeyframe(const double &weight_factor = 0.03);
};

} //tracking namespace
} // end namespace

#endif // _MC-VEO_TRACKER_HPP_
