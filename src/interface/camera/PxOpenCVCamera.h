/*=====================================================================

 PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

 (c) 2011 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

 This file is part of the PIXHAWK project

 PIXHAWK is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 PIXHAWK is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

 ======================================================================*/

/**
 * @file
 *   @brief Definition of Point Grey Firefly MV interface
 *
 *   @author Pascal Dufour <mavteam@student.ethz.ch>
 *   @author Lionel Heng <hengli@inf.ethz.ch>
 *
 */

#ifndef PXOPENCVCAMERA_H
#define PXOPENCVCAMERA_H

#include "PxCamera.h"
#include "opencv2/highgui/highgui.hpp"

class PxOpenCVCamera : public PxCamera
{
public:
	explicit PxOpenCVCamera(cv::VideoCapture* camera);
	~PxOpenCVCamera();

	bool init(void);
	void destroy(void);

	bool setConfig(const PxCameraConfig& config, bool master = true);

	bool start(void);
	bool stop(void);

	bool grabFrame(cv::Mat& image, uint32_t& skippedFrames,
				   uint32_t& sequenceNum);

private:
	bool setExternalTrigger(void);
	bool setStrobe(uint32_t pin);
	bool setFrameRate(float frameRate);
	bool setMode(PxCameraConfig::Mode mode);
	bool setExposureTime(uint32_t exposureTime);
	bool setGain(uint32_t gain);
	bool setGamma(uint32_t gamma);
	bool setBrightness(uint32_t brightness);
    bool setResolution(uint32_t resW, uint32_t resH); 

	cv::VideoCapture* camera; //camera connected to this capture structure

	friend class PxFireflyStereoCamera;
};

#endif
