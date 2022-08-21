/*
#pragma once

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
#include "visualOdometry.h"



	//int frame_id = 0;

	
	// initial pose variables
	cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat frame_pose = cv::Mat::eye(4, 4, CV_64F);

	cv::Mat trajectory = cv::Mat::zeros(600, 1200, CV_8UC3);

	// set of features currently tracked
	//FeatureSet currentVOFeatures;

	// for timing code
	//clock_t t_a, t_b;
	//clock_t t_1, t_2;



	*/