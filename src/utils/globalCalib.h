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
#include "mc-veo/utils/settings.h"
#include "mc-veo/utils/NumType.h"

namespace dso
{
	extern int wG[PYR_LEVELS], hG[PYR_LEVELS];
	extern float fxG[PYR_LEVELS], fyG[PYR_LEVELS],
		  cxG[PYR_LEVELS], cyG[PYR_LEVELS];

	extern float fxiG[PYR_LEVELS], fyiG[PYR_LEVELS],
		  cxiG[PYR_LEVELS], cyiG[PYR_LEVELS];

	extern Eigen::Matrix3f KG[PYR_LEVELS],KiG[PYR_LEVELS];

	extern float wM3G;
	extern float hM3G;

	void setGlobalCalib(int w, int h, const Eigen::Matrix3f &K );
}
