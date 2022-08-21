#pragma once
#ifndef UTILS_H
#define UTILS_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>


#include "feature.h"
#include "matrix.h"

bool isRotationMatrix(cv::Mat& R);

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R);

void integrateOdometryStereo(int frame_id, cv::Mat& frame_pose, const cv::Mat& rotation,
	const cv::Mat& translation_stereo);
#endif // !UTILS_H
