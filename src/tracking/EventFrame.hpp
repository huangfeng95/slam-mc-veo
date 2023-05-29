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

#ifndef _MC_VEO_EVENT_FRAME_HPP_
#define _MC_VEO_EVENT_FRAME_HPP_

#include <mc-veo/utils/Utils.hpp>
#include <mc-veo/utils/Colormap.hpp>

#include <mc-veo/tracking/Config.hpp>

namespace mc_veo {
namespace tracking {
    class EventFrame
    {
        public:
            static constexpr float log_eps = 1e-06;
        public:
            /* unique id **/
            uint64_t idx;
            /** Desired Image dimension **/
            uint16_t height, width;
            /** Time stamps **/
            base::Time first_time, last_time, time, delta_time;
            /** Undistortion Maps (inverse and forward mapping) **/
            cv::Mat mapx, fwd_mapx, mapy, fwd_mapy;
            /** Rescale out img order: (W x H) like in cv::Size**/
            std::array<double, 2> out_scale;
            /** Events Coordinates and normalize coord **/
            std::vector<cv::Point2d> coord, undist_coord;
            Eigen::MatrixXd matrix_coord, matrix_coord3d, matrix_warped_coord_3d, matrix_warped_coord;
            std::vector<cv::Point2d> warped_coord;
            std::vector<Eigen::Vector3d> warped_coord_3d;
            cv::Mat Ix;
            cv::Mat Iy;
            Eigen::VectorXd Ix_interp;
            Eigen::VectorXd Iy_interp;
            Eigen::VectorXd xp;
            Eigen::VectorXd yp;
            Eigen::VectorXd zp;
            Eigen::VectorXd xp_zp;
            Eigen::VectorXd yp_zp;
            Eigen::VectorXd l_zp;
            Eigen::VectorXd warped_image_delta_t;
            Eigen::MatrixXd Jacobian;
            Eigen::VectorXd Gradient;
            double nu_event;
            Eigen::Vector3d update_angular_velocity;
            Eigen::Vector3d update_linear_velocity;
            /** Events polarities **/
            std::vector<int8_t> pol;
            /** Distortion model information **/
            std::string distortion_model;
            /** Intrisic and rectification matrices **/
            cv::Mat K, D, K_ref, R_rect;
            /** Event Frame pose **/
            ::base::Affine3d T_w_ef;
            /** Event frame (integration of events) no normalized **/
            std::vector<cv::Mat> frame;
            /** Normalized event frame in std vector for optimization **/
            std::vector< std::vector<double> > event_frame; // event_frame = frame / norm
            /** Norm of the event frame **/
            std::vector<double> norm;

        public:
            /** @brief Default constructor **/
            EventFrame(const ::mc_veo::calib::Camera &cam, const ::mc_veo::calib::Camera &newcam,  const std::string &distortion_model="radtan");

            EventFrame(const uint64_t &idx, const std::vector<base::samples::Event> &events,
                    const ::mc_veo::calib::CameraInfo &cam_info, const int &num_levels = 1,
                    const ::base::Affine3d &T=::base::Affine3d::Identity(),
                    const cv::Size &out_size = cv::Size(0, 0));

            /** @brief Default constructor **/
            EventFrame(const uint64_t &idx, const std::vector<base::samples::Event> &events, const uint16_t height, const uint16_t width,
                        cv::Mat &K, cv::Mat &D, cv::Mat &R_rect, cv::Mat &P, const std::string distortion_model="radtan", const int &num_levels = 1,
                        const ::base::Affine3d &T=::base::Affine3d::Identity(), const cv::Size &out_size = cv::Size(0, 0));

            /** @brief Insert new Eventframe **/
            void create(const uint64_t &idx, const std::vector<base::samples::Event> &events,
                    const ::mc_veo::calib::CameraInfo &cam_info, const int &num_levels = 1,
                    const ::base::Affine3d &T=::base::Affine3d::Identity(),
                    const cv::Size &out_size = cv::Size(0, 0));

            void create(const uint64_t &idx, const std::vector<base::samples::Event> &events,
                        const uint16_t height, const uint16_t width, const int &num_levels = 1,
                        const ::base::Affine3d &T=::base::Affine3d::Identity(), const cv::Size &out_size = cv::Size(0, 0));

            std::vector<uint32_t> GetValidIndexFromEvent(const Eigen::MatrixXd & event);
            void GetWarpedEventPoint(const Eigen::VectorXd &vdelta_t,
                    const Eigen::Vector3d &v_linear=Eigen::Vector3d::Zero(), const Eigen::Vector3d &v_angular=Eigen::Vector3d::Zero(),
                    const ::base::Affine3d &pose_w_ef=::base::Affine3d::Identity());
            void DeriveErrAnalytic(const Eigen::VectorXd &vdelta_t,
                    const Eigen::Vector3d &v_linear=Eigen::Vector3d::Zero(), const Eigen::Vector3d &v_angular=Eigen::Vector3d::Zero(),
                    const ::base::Affine3d &pose_w_ef=::base::Affine3d::Identity());
            void create_aligned(const uint64_t &idx, const std::vector<base::samples::Event> &events,
                    const ::mc_veo::calib::CameraInfo &cam_info, const int &num_levels = 1,
                    const cv::Size &out_size = cv::Size(0, 0), 
                    const ::base::Affine3d &T=::base::Affine3d::Identity());
                std::vector<uint32_t> GetValidIndexFromEvent(const std::vector<cv::Point2d> & event);
            void GetWarpedEventPoint(const std::vector<double> &vdelta_t,
                    const std::vector<cv::Point2d> &eventIn, std::vector<Eigen::Vector3d> &eventOut_3d, std::vector<cv::Point2d> &eventOut,
                    const Eigen::Vector3d &v_linear=Eigen::Vector3d::Zero(), const Eigen::Vector3d &v_angular=Eigen::Vector3d::Zero(),
                    const ::base::Affine3d &pose_w_ef=::base::Affine3d::Identity());
            void DeriveErrAnalytic(const std::vector<double> &vdelta_t,
                    const Eigen::Vector3d &v_linear=Eigen::Vector3d::Zero(), const Eigen::Vector3d &v_angular=Eigen::Vector3d::Zero(),
                    const ::base::Affine3d &pose_w_ef=::base::Affine3d::Identity());

            void clear();

            cv::Mat viz(size_t id=0, bool color = false);

            cv::Mat getEventFrame(const size_t &id=0);

            cv::Mat getEventFrameViz(const size_t &id=0, bool color = false);

            void setPose(const ::base::Transform3d& pose);

            ::base::Transform3d& getPose();

            ::base::Matrix4d getPoseMatrix();

            std::pair<Eigen::Vector3d, Eigen::Quaterniond> getTransQuater();

            cv::Mat epilinesViz(const std::vector<cv::Point2d> &coord, const cv::Mat &F, const size_t &skip_amount = 10.0);

            cv::Mat pyramidViz(const bool &color = false);

    };

} //tracking namespace
} // end namespace

#endif // _MC_VEO_EVENT_FRAME_HPP_
