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

#ifndef MC_VEO_MC_VEO_H
#define MC_VEO_MC_VEO_H

/** Base types **/
#include <base/Eigen.hpp>

/** Settings **/
#include <mc-veo/utils/settings.h>

/** Utils **/
#include <mc-veo/utils/Utils.hpp>
#include <mc-veo/utils/Calib.hpp>
#include <mc-veo/utils/NumType.h>
#include <mc-veo/utils/globalFuncs.h>
#include <mc-veo/utils/globalCalib.h>
#include <mc-veo/utils/Undistort.h>
#include <mc-veo/utils/ImageAndExposure.h>
#include <mc-veo/utils/FrameShell.h>
#include <mc-veo/utils/IndexThreadReduce.h>

/** I/O **/
#include <mc-veo/io/ImageRW.h>
#include <mc-veo/io/ImageConvert.h>
#include <mc-veo/io/OutputMaps.h>

/** Initialization **/
#include <mc-veo/init/CoarseInitializer.h>

/** Event Tracker (MC-VEO) **/
#include <mc-veo/tracking/EventFrame.hpp>
#include <mc-veo/tracking/KeyFrame.hpp>
#include <mc-veo/tracking/Tracker.hpp>

/** Frame Tracker (DSO) **/
#include <mc-veo/tracking/Residuals.h>
#include <mc-veo/tracking/HessianBlocks.h>
#include <mc-veo/tracking/ImmaturePoint.h>
#include <mc-veo/tracking/CoarseTracker.h>

/** Mapping **/
#include <mc-veo/mapping/Config.hpp>
#include <mc-veo/mapping/Types.hpp>
#include <mc-veo/mapping/PixelSelector.h>

/** Bundles **/
#include <mc-veo/bundles/Config.hpp>
#include <mc-veo/bundles/EnergyFunctional.h>
#include <mc-veo/bundles/EnergyFunctionalStructs.h>
#include <mc-veo/bundles/MatrixAccumulators.h>

#endif //MC_VEO_MC_VEO_H
