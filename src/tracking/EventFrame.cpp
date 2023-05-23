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

#include "EventFrame.hpp"

using namespace mc-veo::tracking;

EventFrame::EventFrame(const ::mc-veo::calib::Camera &cam, const ::mc-veo::calib::Camera &newcam, const std::string &distortion_model)
{
    this->K = cam.K.clone();
    this->D = cam.D.clone();
    this->R_rect = newcam.R * cam.R.t();
    this->K_ref =  newcam.K.clone();
    this->distortion_model = distortion_model;

    if (distortion_model.compare("equidistant") != 0)
    {
        cv::initUndistortRectifyMap(this->K, this->D,
                            this->R_rect, this->K_ref,
                            newcam.size,//undistorted image size
                            CV_32FC1,
                            this->mapx, this->mapy);
    }
    else
    {
        cv::fisheye::initUndistortRectifyMap(this->K, this->D,
                            this->R_rect, this->K_ref,
                            newcam.size,//undistorted image size
                            CV_32FC1,
                            this->mapx, this->mapy);
    }
    /** Create look-up table for all possible coordinates in events
     * We need to do this since initUndistortRectifyMap create the inverse
     * mapping for remap. Which is not useful for undistroting events.
     * We do this only once in the constructor **/
    std::vector<cv::Point2f> coord, undist_coord; // H x W points
    for (int y=0; y<newcam.size.height; ++y)
    {
        for (int x=0; x<newcam.size.width; ++x)
        {
            coord.push_back(cv::Point2f(x, y));
        }
    }

    if (distortion_model.compare("equidistant") != 0)
    {
        cv::undistortPoints(coord, undist_coord, this->K, this->D, this->R_rect, this->K_ref);
    }
    else
    {
        cv::fisheye::undistortPoints(coord, undist_coord, this->K, this->D, this->R_rect, this->K_ref);
    }

    /** Reshape to get the forward maps **/
    this->fwd_mapx = cv::Mat_<float>::eye(newcam.size.height, newcam.size.width);
    this->fwd_mapy = cv::Mat_<float>::eye(newcam.size.height, newcam.size.width);
    int idx = 0; for (auto &it : undist_coord)
    {
        float y = idx/newcam.size.width;
        float x = idx - ((int)y * newcam.size.width);
        this->fwd_mapx.at<float>(y, x) = it.x;
        this->fwd_mapy.at<float>(y, x) = it.y;
        idx++;
    }

    /* Scale to one (no resize) **/
    this->out_scale[0] = 1.0; this->out_scale[1] = 1.0;

    /** Check if the input image should be downscaled **/
    if ((newcam.out_size.height != 0 || newcam.out_size.width != 0) || (newcam.out_size.height != newcam.size.height || newcam.out_size.width != newcam.size.width))
    {
        /** Downrescale the input **/
        this->out_scale[0] = (double)newcam.size.width / (double)newcam.out_size.width;
        this->out_scale[1] = (double)newcam.size.height / (double)newcam.out_size.height;
        this->K.at<double>(0,0) /=  this->out_scale[0]; this->K.at<double>(1,1) /=  this->out_scale[1];
        this->K.at<double>(0,2) /=  this->out_scale[0]; this->K.at<double>(1,2) /=  this->out_scale[1];
        this->K_ref.at<double>(0,0) /=  this->out_scale[0]; this->K_ref.at<double>(1,1) /=  this->out_scale[1];
        this->K_ref.at<double>(0,2) /=  this->out_scale[0]; this->K_ref.at<double>(1,2) /=  this->out_scale[1];
    }

    std::cout<<"** EVENTFRAME: CAMERA CALIB: **"<<std::endl;
    std::cout<<"Model: "<<distortion_model<<std::endl;
    std::cout<<"Size: "<<newcam.size<<std::endl;
    std::cout<<"Out Size: "<<newcam.out_size<<std::endl;
    std::cout<<"K:\n"<<this->K<<std::endl;
    std::cout<<"D:\n"<<this->D<<std::endl;
    std::cout<<"R:\n"<<this->R_rect<<std::endl;
    std::cout<<"K_ref:\n"<<this->K_ref<<std::endl;
    std::cout<<"OUT SCALE ["<<this->out_scale[0]<<","<<this->out_scale[1]<<"]"<<std::endl;

    std::cout<<"mapx: "<<this->mapx.rows<<" x "<<this->mapx.cols;
    std::cout<<" forward mapx: "<<this->fwd_mapx.rows<<" x "<<this->fwd_mapx.cols<<std::endl;
    std::cout<<"mapy: "<<this->mapy.rows<<" x "<<this->mapy.cols;
    std::cout<<" forward mapy: "<<this->fwd_mapy.rows<<" x "<<this->fwd_mapy.cols<<std::endl;
}

EventFrame::EventFrame(const uint64_t &idx, const std::vector<base::samples::Event> &events,
                    const ::mc-veo::calib::CameraInfo &cam_info, const int &num_levels,
                    const ::base::Affine3d &T,
                    const cv::Size &out_size)
{
    cv::Mat K, D, R_rect, P;
    R_rect  = cv::Mat_<double>::eye(3, 3);
    K = cv::Mat_<double>::eye(3, 3);
    K.at<double>(0,0) = cam_info.intrinsics[0];
    K.at<double>(1,1) = cam_info.intrinsics[1];
    K.at<double>(0,2) = cam_info.intrinsics[2];
    K.at<double>(1,2) = cam_info.intrinsics[3];

    D = cv::Mat_<double>::zeros(4, 1);
    for (size_t i=0; i<cam_info.D.size(); ++i)
    {
        D.at<double>(i, 0) = cam_info.D[i];
    }

    if (cam_info.P.size() == 12)
    {
        P = cv::Mat_<double>::zeros(4, 4);
        for (auto row=0; row<P.rows; ++row)
        {
            for (auto col=0; col<P.cols; ++col)
            {
                P.at<double>(row, col) = cam_info.P[(P.cols*row)+col];
            }
        }
    }

    if (cam_info.R.size() == 9)
    {
        for (auto row=0; row<R_rect.rows; ++row)
        {
            for (auto col=0; col<R_rect.cols; ++col)
            {
                R_rect.at<double>(row, col) = cam_info.R[(R_rect.cols*row)+col];
            }
        }
    }

    (*this) = EventFrame(idx, events, cam_info.height, cam_info.width, K, D, R_rect, P, cam_info.distortion_model, num_levels, T, out_size);
}

EventFrame::EventFrame(const uint64_t &idx, const std::vector<base::samples::Event> &events, const uint16_t height, const uint16_t width,
            cv::Mat &K, cv::Mat &D, cv::Mat &R_rect, cv::Mat &P, const std::string distortion_model, const int &num_levels,
            const ::base::Affine3d &T, const cv::Size &out_size)
            :idx(idx), height(height), width(width), distortion_model(distortion_model), T_w_ef(T)
{

    if (P.total()>0)
        this->K_ref = P(cv::Rect(0,0,3,3));
    if (R_rect.total()>0)
        this->R_rect = R_rect.clone();

    /** Distortion **/
    if (K.total() > 0 && D.total() > 0)
    {
        this->K = K.clone();
        this->D = D.clone();
        if (this->K_ref.total() == 0)
        {
            if (distortion_model.compare("equidistant") != 0)
            {
                this->K_ref = cv::getOptimalNewCameraMatrix(this->K, this->D, cv::Size(width, height), 0.0);
            }
            else
            {
                cv::fisheye::estimateNewCameraMatrixForUndistortRectify(this->K, this->D, cv::Size(width, height), this->R_rect, this->K_ref);
            }
        }
    }
    else
    {
        this->K_ref = K.clone();
    }


    /** Get the events and polarity **/
    for (auto it=events.begin(); it!=events.end(); ++it)
    {
        this->coord.push_back(cv::Point2d(it->x, it->y));
        this->pol.push_back((it->polarity)?1:-1);
        if (it == events.begin())
            this->first_time = it->ts;
        else if ((it+1) == events.end())
            this->last_time = it->ts;
    }

    if (first_time.toMicroseconds() > last_time.toMicroseconds())
    {
        std::string error_message = std::string("[EVENT_FRAME] FATAL ERROR Event time[0] > event time [N-1] ");
        throw std::runtime_error(error_message);
    }

    /** Frame time as the median event time **/
    auto it = events.begin(); std::advance(it, events.size()/2);
    this->time = it->ts;

    /** Delta time of thsi event frame **/
    this->delta_time = (last_time - first_time);

    /** Undistort the points **/
    if (distortion_model.compare("equidistant") != 0)
    {
        cv::undistortPoints(this->coord, this->undist_coord, this->K, this->D, this->R_rect, this->K_ref);
    }
    else
    {
        cv::fisheye::undistortPoints(this->coord, this->undist_coord, this->K, this->D, this->R_rect, this->K_ref);
    }

    /**  Compute brightness change event frame (undistorted) per pyramid level **/
    cv::Mat event_img = mc-veo::utils::drawValuesPoints(this->undist_coord, this->pol, height, width, "bilinear", 0.5, true);
    cv::Size size = event_img.size();

    /** Check if the input image should be rescale **/
    if ((out_size.height == 0 || out_size.width == 0) || (out_size.height > size.height || out_size.width > size.width))
    {
        this->out_scale[0] = 1.0; this->out_scale[1] = 1.0;
        std::cout<<"[EVENT_FRAME] out_scale["<<this->out_scale[0]<<","<<this->out_scale[1]<<"] out_size: "<<size<<std::endl;
    }
    else
    {
        /** Rescale the input **/
        this->out_scale[0] = (double)size.width / (double)out_size.width;
        this->out_scale[1] = (double)size.height / (double)out_size.height;
        std::cout<<"[EVENT_FRAME] out_scale["<<this->out_scale[0]<<","<<this->out_scale[1]<<"] out_size: "<<out_size<<std::endl;
        this->width /= this->out_scale[0]; this->height /= this->out_scale[1];
        this->K.at<double>(0,0) /=  this->out_scale[0]; this->K.at<double>(1,1) /=  this->out_scale[1];
        this->K.at<double>(0,2) /=  this->out_scale[0]; this->K.at<double>(1,2) /=  this->out_scale[1];
        this->K_ref.at<double>(0,0) /=  this->out_scale[0]; this->K_ref.at<double>(1,1) /=  this->out_scale[1];
        this->K_ref.at<double>(0,2) /=  this->out_scale[0]; this->K_ref.at<double>(1,2) /=  this->out_scale[1];
        cv::resize(event_img, event_img, out_size, cv::INTER_CUBIC);
        size = event_img.size();
    }

    this->frame.push_back(event_img);
    for (int i=1; i<num_levels; ++i)
    {
        cv::Mat event_img_dilate, event_img_erode;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*i + 1, 2*i + 1), cv::Point(i, i));
        cv::dilate(event_img, event_img_dilate, element);
        cv::erode(event_img, event_img_erode, element);
        this->frame.push_back(event_img_dilate + event_img_erode);
    }

    /** Norm of the event frame **/
    for (auto f=frame.begin(); f!=frame.end(); ++f)
    {
        double n_f = cv::norm (*f);
        this->norm.push_back(n_f);
    }

    /** Normalized event frame **/
    int id = 0; this->event_frame.clear();
    for (auto &it : this->frame)
    {
        std::vector<double> frame_item;
        for (int row=0; row<it.rows; row++)
        {
            for (int col=0; col<it.cols;col++)
            {
                /** When using PhotometricError cost function  **/
                frame_item.push_back(it.at<double>(row,col)/this->norm[id]);
                /** When using PhotometricErrorNC cost function **/
                //frame_item.push_back(it.at<double>(row,col));
            }
        }
        this->event_frame.push_back(frame_item);
        ++id;
    }
    std::cout<<"[EVENT_FRAME] Created ID["<<this->idx<<"] with: "<<events.size()
                <<" events. start time "<<first_time.toMicroseconds()<<" end time "
                <<last_time.toMicroseconds()<<std::endl;
    std::cout<<"[EVENT_FRAME] event frame ["<<std::addressof(this->event_frame)<<"] size:"<<this->event_frame.size()<<std::endl;

}

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
}

// warping function
void EventFrame::GetWarpedEventPoint(const Eigen::VectorXd &vdelta_t, const Eigen::Vector3d &v_linear, const Eigen::Vector3d &v_angular, const ::base::Affine3d &pose_w_ef){

    Eigen::Vector3d tmp_V(v_linear);
    
    Eigen::MatrixXd x_ang_vel_hat, x_ang_vel_hat_square;
    this->matrix_warped_coord.resize(this->matrix_coord.rows(), 2);
    this->matrix_warped_coord_3d.resize(this->matrix_coord.rows(), 3);
    x_ang_vel_hat.resize(this->matrix_coord.rows(), 3);
    x_ang_vel_hat_square.resize(this->matrix_coord.rows(), 3);

    x_ang_vel_hat.col(0) = - v_angular(2) * this->matrix_coord3d.col(1) + v_angular(1) * this->matrix_coord3d.col(2);
    x_ang_vel_hat.col(1) = + v_angular(2) * this->matrix_coord3d.col(0) - v_angular(0) * this->matrix_coord3d.col(2);
    x_ang_vel_hat.col(2) = - v_angular(1) * this->matrix_coord3d.col(0) + v_angular(0) * this->matrix_coord3d.col(1);

    x_ang_vel_hat_square.col(0) = - v_angular(2) * x_ang_vel_hat.col(1) + v_angular(1) * x_ang_vel_hat.col(2);
    x_ang_vel_hat_square.col(1) = + v_angular(2) * x_ang_vel_hat.col(0) - v_angular(0) * x_ang_vel_hat.col(2);
    x_ang_vel_hat_square.col(2) = - v_angular(1) * x_ang_vel_hat.col(0) + v_angular(0) * x_ang_vel_hat.col(1);

    auto d_t = vdelta_t.array();
    this->matrix_warped_coord_3d = this->matrix_coord3d
                        + Eigen::MatrixXd( x_ang_vel_hat.array().colwise()
                        * d_t.array()
                        + x_ang_vel_hat_square.array().colwise() 
                        * (0.5f * d_t.array().square()) )
                        + vdelta_t * tmp_V.transpose();

    for (int ev_idx = 0; ev_idx < this->matrix_warped_coord_3d.rows(); ev_idx++)
    {
        Eigen::Vector3d p = this->matrix_warped_coord_3d.row(ev_idx);
        
        Eigen::Vector2d p2d(p[0]/p[2], p[1]/p[2]);
        p2d[0] = this->K.at<double>(0, 0)*p2d[0] + this->K.at<double>(0, 2);
        p2d[1] = this->K.at<double>(1, 1)*p2d[1] + this->K.at<double>(1, 2);
        this->matrix_warped_coord.row(ev_idx) = p2d;
    }
}

std::vector<uint32_t> EventFrame::GetValidIndexFromEvent(const Eigen::MatrixXd & event){
    std::vector<uint32_t> valid;
    for (uint32_t pts_iter = 0; pts_iter < event.rows(); pts_iter++){
        if (event.row(pts_iter)[0] <= 0 || event.row(pts_iter)[0] >= this->width-1 || event.row(pts_iter)[1] <= 0 || event.row(pts_iter)[1] >= this->height-1)
        {
        }
        else
        {
            valid.push_back(pts_iter);
        }
    }
    return valid;
}

// Compute Jacobian matrix
// update Ix, Iy, Jacobian, warped_image_valid
void EventFrame::DeriveErrAnalytic(const Eigen::VectorXd &vdelta_t, const Eigen::Vector3d &v_linear, const Eigen::Vector3d &v_angular, const ::base::Affine3d &pose_w_ef)
{
    GetWarpedEventPoint(vdelta_t, v_linear, v_angular, pose_w_ef);

    cv::Mat event_img_aligned = mc-veo::utils::drawValuesPoints(this->matrix_warped_coord, this->pol, this->height, this->width, "bilinear",
                                0.5, false);
    
    cv::Sobel(event_img_aligned, Ix, CV_64FC1, 1, 0);
    cv::Sobel(event_img_aligned, Iy, CV_64FC1, 0, 1);

    std::vector<uint32_t> e_valid = GetValidIndexFromEvent(this->matrix_warped_coord);
    
    uint32_t valid_size = e_valid.size();
    
    xp.resize(valid_size);
    yp.resize(valid_size);
    zp.resize(valid_size);
    Ix_interp.resize(valid_size);
    Iy_interp.resize(valid_size);
    warped_image_delta_t.resize(valid_size);
    Jacobian.resize(valid_size, 6);
    
    int16_t coord_x;
    int16_t coord_y;
    uint32_t e_valid_v_iter;
    for (uint32_t v_iter = 0; v_iter < valid_size; v_iter++)
    {
        e_valid_v_iter = e_valid[v_iter];
        
        coord_x = std::round(this->matrix_warped_coord.row(e_valid_v_iter)[0]);
        coord_y = std::round(this->matrix_warped_coord.row(e_valid_v_iter)[1]);
        
        xp(v_iter) = this->matrix_warped_coord_3d.row(e_valid_v_iter)[0];
        yp(v_iter) = this->matrix_warped_coord_3d.row(e_valid_v_iter)[1];
        zp(v_iter) = this->matrix_warped_coord_3d.row(e_valid_v_iter)[2];
        
        Ix_interp(v_iter) = Ix.at<double>(coord_y, coord_x);
        Iy_interp(v_iter) = Iy.at<double>(coord_y, coord_x);
        
        warped_image_delta_t(v_iter) = - vdelta_t[e_valid_v_iter] 
                                * event_img_aligned.at<double>(coord_y, coord_x);       //负号？
        
    }
    Ix_interp *= this->K.at<double>(0, 0);
    Iy_interp *= this->K.at<double>(1, 1);

    xp_zp = xp.array() / zp.array();
    yp_zp = yp.array() / zp.array();
    l_zp = 1 / zp.array();
    Eigen::ArrayXd xx_zz = xp_zp.array() * xp_zp.array();
    Eigen::ArrayXd yy_zz = yp_zp.array() * yp_zp.array();
    Eigen::ArrayXd xy_zz = xp_zp.array() * yp_zp.array();
    Jacobian.col(0) = -(Ix_interp.array() * xy_zz) 
                    - (Iy_interp.array() * (1 + yy_zz));
    Jacobian.col(1) = (Ix_interp.array() * (1 + xx_zz)) 
                    + (Iy_interp.array() * xy_zz);
    Jacobian.col(2) = -Ix_interp.array() * yp_zp.array() + Iy_interp.array() * xp_zp.array();
    Jacobian.col(3) = (Ix_interp.array() * l_zp.array());
    Jacobian.col(4) = (Iy_interp.array() * l_zp.array());
    Jacobian.col(5) = -(Ix_interp.array() * xp_zp.array() * l_zp.array())
                     - (Iy_interp.array() * yp_zp.array() * l_zp.array());

}

void EventFrame::create_aligned(const uint64_t &idx, const std::vector<base::samples::Event> &events,
                    const ::mc-veo::calib::CameraInfo &cam_info, const int &num_levels,
                    const cv::Size &out_size, const ::base::Affine3d &T)
{
    /** Clean before inserting**/
    this->clear();

    /** Input idx and size **/
    this->idx = idx; this->height = cam_info.height; this->width = cam_info.width; this->T_w_ef = T;

    /** Get the coordinates, undistort coordinates, events and polarity **/
    for (auto it=events.begin(); it!=events.end(); ++it)
    {
        this->coord.push_back(cv::Point2d(it->x, it->y));
        this->undist_coord.push_back(cv::Point2d(this->fwd_mapx.at<float>(it->y, it->x),
                                                 this->fwd_mapy.at<float>(it->y, it->x)));
        this->pol.push_back((it->polarity)?1:-1);
        if (it == events.begin())
            this->first_time = it->ts;
        else if ((it+1) == events.end())
            this->last_time = it->ts;
    }

    if (first_time.toMicroseconds() > last_time.toMicroseconds())
    {
        std::string error_message = std::string("[EVENT_FRAME] FATAL ERROR Event time[0] > event time [N-1] ");
        throw std::runtime_error(error_message);
    }

    /** Frame time as the median event time **/
    auto it = events.begin(); std::advance(it, events.size()/2);
    this->time = it->ts;

    /** Delta time of this event frame **/
    this->delta_time = (last_time - first_time);

    this->matrix_coord.resize(this->undist_coord.size(), 2);
    this->matrix_coord3d.resize(this->undist_coord.size(), 3);
    size_t ev_idx = 0;
    for (auto it=this->undist_coord.begin(); it!=this->undist_coord.end(); ++it)
    {
        Eigen::Vector2d p;
        p[0] = (*it).x;
        p[1] = (*it).y;
        
        this->matrix_coord.row(ev_idx) = p;
        Eigen::Vector3d p3d;
        p3d[0] = ((*it).x - this->K.at<double>(0, 2))/this->K.at<double>(0, 0);
        p3d[1] = ((*it).y - this->K.at<double>(1, 2))/this->K.at<double>(1, 1);
        p3d[2] = 1.0;
        
        this->matrix_coord3d.row(ev_idx) = p3d;
        ev_idx++;
    }

    Eigen::VectorXd delta_t;
    delta_t.resize(events.size());
    int t_idx = 0;
    for (auto it=events.begin(); it!=events.end(); ++it)
    {
        delta_t[t_idx] = (this->time.toSeconds() - it->ts.toSeconds());
        t_idx++;
    }

    Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    ::base::Affine3d position_compensator = ::base::Affine3d::Identity();
    Gradient.resize(6);
    nu_event = 0;

    for (int i = 0; i < 25; i++){
        DeriveErrAnalytic(delta_t, linear_velocity, angular_velocity, position_compensator);
        Gradient = Jacobian.transpose() * warped_image_delta_t;
        // RMS-prop optimizer
        nu_event = 0.995 * nu_event
                + (1.0 - 0.995) * (double)(Gradient.transpose() * Gradient);
        update_angular_velocity = - 0.05 / std::sqrt(nu_event + 1e-8) * Gradient.head<3>();
        update_linear_velocity = - 0.05 / std::sqrt(nu_event + 1e-8) * Gradient.tail<3>();
        angular_velocity = update_angular_velocity + angular_velocity;
        linear_velocity = update_linear_velocity + linear_velocity;
        
    }

    GetWarpedEventPoint(delta_t, linear_velocity, angular_velocity, position_compensator);

    cv::Mat event_img = mc-veo::utils::drawValuesPoints(this->matrix_warped_coord, this->pol, this->height, this->width, "bilinear",
                                0.5, false);
    
    cv::Size size = event_img.size();
    
    if (this->out_scale[0] != 1 || this->out_scale[1] != 1)
    {
        this->width /= this->out_scale[0]; this->height /= this->out_scale[1];
        cv::resize(event_img, event_img, out_size, cv::INTER_CUBIC);
        size = event_img.size();
    }

    this->frame.push_back(event_img);
    for (int i=1; i<num_levels; ++i)
    {
        cv::Mat event_img_dilate, event_img_erode;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*i + 1, 2*i + 1), cv::Point(i, i));
        cv::dilate(event_img, event_img_dilate, element);
        cv::erode(event_img, event_img_erode, element);
        this->frame.push_back(event_img_dilate + event_img_erode);
    }

    /** Norm of the event frame **/
    for (auto f=frame.begin(); f!=frame.end(); ++f)
    {
        double n_f = cv::norm (*f);
        this->norm.push_back(n_f);
    }

    /** Normalized event frame **/
    int id = 0; this->event_frame.clear();
    for (auto &it : this->frame)
    {
        std::vector<double> frame_item;
        for (int row=0; row<it.rows; row++)
        {
            for (int col=0; col<it.cols;col++)
            {
                /** When using PhotometricError cost function  **/
                frame_item.push_back(it.at<double>(row,col)/this->norm[id]);
            }
        }
        this->event_frame.push_back(frame_item);
        ++id;
    }
}

// warping function
void EventFrame::GetWarpedEventPoint(const std::vector<double> &vdelta_t, const std::vector<cv::Point2d> &eventIn, std::vector<Eigen::Vector3d> &eventOut_3d, std::vector<cv::Point2d> &eventOut, const Eigen::Vector3d &v_linear, const Eigen::Vector3d &v_angular, const ::base::Affine3d &pose_w_ef){

    eventOut.clear();
    eventOut_3d.clear();
    double delta_t;
    Eigen::Quaterniond tmp_Q;
    Eigen::Vector3d un_gyr(v_angular);

    Eigen::Vector3d tmp_V(v_linear);
    Eigen::Vector3d tmp_P;
    
    size_t ev_idx = 0;
    for (auto it=eventIn.begin(); it!=eventIn.end(); ++it)
    {
        delta_t = vdelta_t[ev_idx];
        tmp_Q = deltaQ(un_gyr * delta_t);
        tmp_Q.normalize();
        tmp_P = tmp_V * delta_t;

        Eigen::Vector3d p;
        p[0] = ((*it).x - this->K.at<double>(0, 2))/this->K.at<double>(0, 0);
        p[1] = ((*it).y - this->K.at<double>(1, 2))/this->K.at<double>(1, 1);
        p[2] = 1.0;
        p = tmp_Q.toRotationMatrix() * p + tmp_P;

        eventOut_3d.push_back(p);

        cv::Point2d p2d(p[0]/p[2], p[1]/p[2]);
        p2d.x = this->K.at<double>(0, 0)*p2d.x + this->K.at<double>(0, 2);
        p2d.y = this->K.at<double>(1, 1)*p2d.y + this->K.at<double>(1, 2);
        eventOut.push_back(p2d);
        ev_idx++;
    }
}

std::vector<uint32_t> EventFrame::GetValidIndexFromEvent(const std::vector<cv::Point2d> & event){
    std::vector<uint32_t> valid;
    for (uint32_t pts_iter = 0; pts_iter < event.size(); pts_iter++){
        if (event[pts_iter].x <= 0 || event[pts_iter].x >= this->width-1 || event[pts_iter].y <= 0 || event[pts_iter].y >= this->height-1)
        {
        }
        else
        {
            valid.push_back(pts_iter);
        }
    }
    return valid;
}

// Compute Jacobian matrix
// update Ix, Iy, Jacobian, warped_image_valid
void EventFrame::DeriveErrAnalytic(const std::vector<double> &vdelta_t, const Eigen::Vector3d &v_linear, const Eigen::Vector3d &v_angular, const ::base::Affine3d &pose_w_ef)
{
    GetWarpedEventPoint(vdelta_t, this->undist_coord, this->warped_coord_3d, this->warped_coord, v_linear, v_angular, pose_w_ef);

    cv::Mat event_img_aligned = mc-veo::utils::drawValuesPoints(this->warped_coord, this->pol, this->height, this->width, "bilinear",
                                0.5, false);
    
    cv::Sobel(event_img_aligned, Ix, CV_64FC1, 1, 0);
    cv::Sobel(event_img_aligned, Iy, CV_64FC1, 0, 1);

    std::vector<uint32_t> e_valid = GetValidIndexFromEvent(this->warped_coord);
    
    uint32_t valid_size = e_valid.size();
    
    xp.resize(valid_size);
    yp.resize(valid_size);
    zp.resize(valid_size);
    Ix_interp.resize(valid_size);
    Iy_interp.resize(valid_size);
    warped_image_delta_t.resize(valid_size);
    Jacobian.resize(valid_size, 6);
    // Jacobian.resize(valid_size, 3);
    
    int16_t coord_x;
    int16_t coord_y;
    uint32_t e_valid_v_iter;
    for (uint32_t v_iter = 0; v_iter < valid_size; v_iter++)
    {
        e_valid_v_iter = e_valid[v_iter];

        coord_x = std::round(this->warped_coord[e_valid_v_iter].x);
        coord_y = std::round(this->warped_coord[e_valid_v_iter].y);

        xp(v_iter) = this->warped_coord_3d[e_valid_v_iter](0);
        yp(v_iter) = this->warped_coord_3d[e_valid_v_iter](1);
        zp(v_iter) = this->warped_coord_3d[e_valid_v_iter](2);

        Ix_interp(v_iter) = Ix.at<double>(coord_y, coord_x);
        Iy_interp(v_iter) = Iy.at<double>(coord_y, coord_x);

        warped_image_delta_t(v_iter) = - vdelta_t[e_valid_v_iter] 
                                * event_img_aligned.at<double>(coord_y, coord_x); 
        
    }
    Ix_interp *= this->K.at<double>(0, 0);
    Iy_interp *= this->K.at<double>(1, 1);

    xp_zp = xp.array() / zp.array();
    yp_zp = yp.array() / zp.array();
    l_zp = 1 / zp.array();
    Eigen::ArrayXd xx_zz = xp_zp.array() * xp_zp.array();
    Eigen::ArrayXd yy_zz = yp_zp.array() * yp_zp.array();
    Eigen::ArrayXd xy_zz = xp_zp.array() * yp_zp.array();
    Jacobian.col(0) = -(Ix_interp.array() * xy_zz) 
                    - (Iy_interp.array() * (1 + yy_zz));
    Jacobian.col(1) = (Ix_interp.array() * (1 + xx_zz)) 
                    + (Iy_interp.array() * xy_zz);
    Jacobian.col(2) = -Ix_interp.array() * yp_zp.array() + Iy_interp.array() * xp_zp.array();
    Jacobian.col(3) = (Ix_interp.array() * l_zp.array());
    Jacobian.col(4) = (Iy_interp.array() * l_zp.array());
    Jacobian.col(5) = -(Ix_interp.array() * xp_zp.array() * l_zp.array())
                     - (Iy_interp.array() * yp_zp.array() * l_zp.array());

}

void EventFrame::create(const uint64_t &idx, const std::vector<base::samples::Event> &events,
                    const ::mc-veo::calib::CameraInfo &cam_info, const int &num_levels,
                    const ::base::Affine3d &T,
                    const cv::Size &out_size)
{
    return this->create(idx, events, cam_info.height, cam_info.width, num_levels, T, out_size);
}

void EventFrame::create(const uint64_t &idx, const std::vector<base::samples::Event> &events,
            const uint16_t height, const uint16_t width, const int &num_levels,
            const ::base::Affine3d &T, const cv::Size &out_size)
{
    /** Clean before inserting**/
    this->clear();

    /** Input idx and size **/
    this->idx = idx; this->height = height; this->width = width; this->T_w_ef = T;

    /** Get the coordinates, undistort coordinates, events and polarity **/
    for (auto it=events.begin(); it!=events.end(); ++it)
    {
        this->coord.push_back(cv::Point2d(it->x, it->y));
        this->undist_coord.push_back(cv::Point2d(this->fwd_mapx.at<float>(it->y, it->x),
                                                 this->fwd_mapy.at<float>(it->y, it->x)));
        this->pol.push_back((it->polarity)?1:-1);
        if (it == events.begin())
            this->first_time = it->ts;
        else if ((it+1) == events.end())
            this->last_time = it->ts;
    }

    if (first_time.toMicroseconds() > last_time.toMicroseconds())
    {
        std::string error_message = std::string("[EVENT_FRAME] FATAL ERROR Event time[0] > event time [N-1] ");
        throw std::runtime_error(error_message);
    }

    /** Frame time as the median event time **/
    auto it = events.begin(); std::advance(it, events.size()/2);
    this->time = it->ts;

    /** Delta time of this event frame **/
    this->delta_time = (last_time - first_time);

    /**  Compute brightness change event frame (undistorted) per pyramid level **/
    cv::Mat event_img = mc-veo::utils::drawValuesPoints(this->undist_coord, this->pol, height, width, "bilinear", 0.5, true);
    cv::Size size = event_img.size();

    if (this->out_scale[0] != 1 || this->out_scale[1] != 1)
    {
        this->width /= this->out_scale[0]; this->height /= this->out_scale[1];
        cv::resize(event_img, event_img, out_size, cv::INTER_CUBIC);
        size = event_img.size();
    }

    this->frame.push_back(event_img);
    for (int i=1; i<num_levels; ++i)
    {
        cv::Mat event_img_dilate, event_img_erode;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*i + 1, 2*i + 1), cv::Point(i, i));
        cv::dilate(event_img, event_img_dilate, element);
        cv::erode(event_img, event_img_erode, element);
        this->frame.push_back(event_img_dilate + event_img_erode);
    }

    /** Norm of the event frame **/
    for (auto f=frame.begin(); f!=frame.end(); ++f)
    {
        double n_f = cv::norm (*f);
        this->norm.push_back(n_f);
    }

    /** Normalized event frame **/
    int id = 0; this->event_frame.clear();
    for (auto &it : this->frame)
    {
        std::vector<double> frame_item;
        for (int row=0; row<it.rows; row++)
        {
            for (int col=0; col<it.cols;col++)
            {
                /** When using PhotometricError cost function  **/
                frame_item.push_back(it.at<double>(row,col)/this->norm[id]);
                /** When using PhotometricErrorNC cost function **/
                //frame_item.push_back(it.at<double>(row,col));
            }
        }
        this->event_frame.push_back(frame_item);
        ++id;
    }
    std::cout<<"[EVENT_FRAME] Created ID["<<this->idx<<"] with: "<<events.size()
                <<" events. start time "<<first_time.toMicroseconds()<<" end time "
                <<last_time.toMicroseconds()<<std::endl;
    std::cout<<"[EVENT_FRAME] event frame ["<<std::addressof(this->event_frame)<<"] size:"<<this->event_frame.size()<<" image size[0]: "<<this->event_frame[0].size()<<std::endl;

}

void EventFrame::clear()
{
    this->coord.clear();
    this->undist_coord.clear();
    this->pol.clear();
    this->frame.clear();
    this->event_frame.clear();
    this->norm.clear();
}

cv::Mat EventFrame::viz(size_t id, bool color)
{
    assert(id < this->frame.size());
    cv::Mat events_viz;
    double min, max;
    cv::Mat &img = this->frame[id];
    cv::minMaxLoc(img, &min, &max);
    cv::Mat norm_img = (img - min)/(max -min);

    norm_img.convertTo(events_viz, CV_8UC1, 255, 0);
    if (color)
    {
        mc-veo::utils::BlueWhiteRed bwr;
        bwr(events_viz, events_viz);
    }

    return events_viz;
}

cv::Mat EventFrame::getEventFrame(const size_t &id)
{
    /* get the image from the std vector **/
    cv::Mat event_mat = cv::Mat(this->height, this->width, CV_64FC1);

    std::memcpy(event_mat.data, this->event_frame[id].data(), this->event_frame[id].size()*sizeof(double));

    return event_mat;
}

cv::Mat EventFrame::getEventFrameViz(const size_t &id, bool color)
{
    cv::Mat img = this->getEventFrame();

    double min, max;
    cv::minMaxLoc(img, &min, &max);
    double bmax = 0.7 * max;
    cv::Mat norm_img = (img + bmax)/(2.0* bmax);//(img - min)/(max -min); //between 0 - 1

    cv::Mat events_viz;
    norm_img.convertTo(events_viz, CV_8UC1, 255, 0);
    if (color)
    {
        mc-veo::utils::BlueWhiteRed bwr;
        bwr(events_viz, events_viz);
    }

    return events_viz;
}

void EventFrame::setPose(const ::base::Transform3d& pose)
{
    this->T_w_ef = pose;
}

::base::Transform3d& EventFrame::getPose()
{
    return this->T_w_ef;
}

::base::Matrix4d EventFrame::getPoseMatrix()
{
    return this->T_w_ef.matrix();
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> EventFrame::getTransQuater()
{
    return std::make_pair(this->T_w_ef.translation(), Eigen::Quaterniond(this->T_w_ef.rotation()));
}

cv::Mat EventFrame::epilinesViz (const std::vector<cv::Point2d> &coord, const cv::Mat &F, const size_t &skip_amount)
{
    std::vector<cv::Vec3d> lines;
    cv::computeCorrespondEpilines(coord, 1, F, lines);

    cv::Mat img_color = this->getEventFrameViz(0, false);
    cv::cvtColor(img_color, img_color, cv::COLOR_GRAY2RGB);

    auto it_l = lines.begin();
    for(; it_l < lines.end(); it_l+=skip_amount)
    {
        cv::Vec3d &l = *it_l; //lines[1000]
        cv::Scalar color((double)std::rand() / RAND_MAX * 255,
        (double)std::rand() / RAND_MAX * 255,
        (double)std::rand() / RAND_MAX * 255);
        cv::line(img_color, cv::Point(0, -l[2]/l[1]), cv::Point(img_color.cols, -(l[2] + l[0] * img_color.cols) / l[1]), color);
    }

    return img_color;
}

cv::Mat EventFrame::pyramidViz(const bool &color)
{
    size_t n = this->frame.size();
    cv::Mat img = cv::Mat(n * this->height, this->width, CV_64FC1, cv::Scalar(0));

    for (size_t i=0; i<n; ++i)
    {
        cv::Size size = this->frame[i].size();
        this->frame[i].copyTo(img(cv::Rect(0, i*size.height, size.width, size.height)));
    }

    return ::mc-veo::utils::viz(img);
}
