/*
#include "stereo_vo.h"

StereoVO::StereoVO(cv::Mat projectMatrl_, cv::Mat projMatrr_)
{
	projMatrl = projectMatrl_;
	projMatrr = projMatrr_;
}

cv::Mat StereoVO::run()
{
	t_a = clock();
	t_1 = clock();
	std::vector<cv::Point2f> pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1;
	matchingFeatures(imageLeft_t0,imageRight_t0,
		             imageLeft_t1,imageRight_t1,
		             currentVOFeatures,
		             pointsLeft_t0,
		             pointsRight_t0,
		             pointsLeft_t1,
		             pointsRight_t1);

	t_2 = clock();

	// set new images as old images
	imageLeft_t0 = imageLeft_t1;
	imageRight_t0 = imageRight_t1;

	// display visualize feature track
	displayTracking(imageLeft_t1, pointsLeft_t0, pointsLeft_t1);

	if (currentVOFeatures.size() < 5)
	{
		std::cout << "not enough features matched for pose estimation" << std::endl;
		frame_id++;
		return;
	}

	//---------------------------
	// Triangulate 3D Points
	//----------------------------
	cv::Mat points3D_t0, points4D_t0;
	cv::triangulatePoints(projMatrl, projMatrr, pointsLeft_t0, pointsRight_t0, points4D_t0);
	cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

	//-----------------------------
	// Tracking Transformation
	//-----------------------------
	// PnP: computes rotation and translation between pair of images
	trackingFrame2Frame(projMatrl, projMatrr, pointsLeft_t1, points3D_t0, rotation, translation, false);

	//---------------------------
	// Integrating
	//---------------------------
	cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);
	if (abs(rotation_euler[1])<0.1 && abs(rotation_euler[0]) < 0.1 && abs(rotation_euler[2]) < 0.1)
	{
		integrateOdometryStereo(frame_id, frame_pose, rotation, translation);
	}
	else {
		std::cout << "Too Large rotation" << std::endl;
	}
	t_b = clock();
	float frame_time = 1000 * (double)(t_b - t_a) / CLOCKS_PER_SEC;
	float fps = 1000 / frame_time;

	cv::Mat xyz = frame_pose.col(3).clone();
	cv::Mat R = frame_pose(cv::Rect(0, 0, 3, 3));

	if (false)
	{
		float time_matching_features = 1000 * (double)(t_2 - t_1) / CLOCKS_PER_SEC;
		std::cout << "time to match features" << time_matching_features << std::endl;
		std::cout << "time total" << float(t_b - t_a) / CLOCKS_PER_SEC * 1000 << std::endl;

	}


	if (true)
	{
		std::cout << xyz << std::endl;
	}
	frame_id++;
}*/