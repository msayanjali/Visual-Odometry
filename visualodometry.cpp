
#include "visualOdometry.h"
using namespace cv;


void checkValidMatch(std::vector<cv::Point2f>& points, std::vector<cv::Point2f> points_return, std::vector<bool>& status, int threshold)
{
	int offset;
	for (int i = 0; i < points.size(); i++)
	{
		offset = std::max(std::abs(points[i].x - points_return[i].x), std::abs(points[i].y - points_return[i].y));
		if (offset>threshold)
		{
			status.push_back(false);
		}
		else
		{
			status.push_back(true);
		}
	}
}


void removeInvalidPoints(std::vector<cv::Point2f>& points, const std::vector<bool>& status)
{
	int index = 0;
	for (int i = 0; i < status.size(); i++)
	{
		if (status[i]==false)
		{
			points.erase(points.begin() + index);
		}
		else
		{
			index++;
		}
	}
}

void matchingFeatures(cv::Mat& imageLeft_t0, cv::Mat& imageRight_t0,
					  cv::Mat& imageLeft_t1, cv::Mat& imageRight_t1,
					  FeatureSet& currentVOFeatures,
					  std::vector<cv::Point2f>& pointsLeft_t0,
				      std::vector<cv::Point2f>& pointsRight_t0,
					  std::vector<cv::Point2f>& pointsLeft_t1,
					  std::vector<cv::Point2f>& pointsRight_t1)
{
	//----------------------------------------------------------------------
	// Feature detection using FAST
	//----------------------------------------------------------------------
	std::vector<cv::Point2f> pointsLeftReturn_t0;  //feature points to check cicular mathcing validation

	// add new features if current number of features is below a threshold. 
	
	if (currentVOFeatures.size()<2000)
	{
		// append new features with old features
		
		appendNewFeatures(imageLeft_t0, currentVOFeatures);
	}
	
		
	//--------------------------------------------------------------------
	// Feature tracking using KLT tracker, bucketing and circular matching
	//--------------------------------------------------------------------
	int bucket_size = std::min(imageLeft_t0.rows, imageLeft_t0.cols) / 10;
	int features_per_bucket = 1;
	std::cout << "nember of features before bucketing" << currentVOFeatures.points.size() << std::endl;
	
	
	// filter features in currentVOFeatures so that one per bucket
	//bucketingFeatures(imageLeft_t0, currentVOFeatures, bucket_size, features_per_bucket);
	pointsLeft_t0 = currentVOFeatures.points;
	



	circularMatching(imageLeft_t0, imageRight_t0, imageLeft_t1, imageRight_t1,
		pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1, pointsLeftReturn_t0, currentVOFeatures);
	

	// check if circled back points are in range of original points
	std::vector<bool> status;
	checkValidMatch(pointsLeft_t0, pointsLeftReturn_t0, status, 0);
	
	removeInvalidPoints(pointsLeft_t0, status); // can combine into one function
	removeInvalidPoints(pointsLeft_t1, status);
	removeInvalidPoints(pointsRight_t0, status);
	removeInvalidPoints(pointsRight_t1, status);

	std::cout << "number of features after bucketing: " << currentVOFeatures.points.size() << std::endl;

	// update current tracked points
	currentVOFeatures.points = pointsLeft_t1;

	std::cout << "number of features after circular matching: " << currentVOFeatures.points.size() << std::endl;

	
}

void displayTracking(cv::Mat& imageLeft_t1,
	std::vector<cv::Point2f>& pointsLeft_t0,
	std::vector<cv::Point2f>& pointsLeft_t1)
{
	// -----------------------------------------
	// Display feature racking
	// -----------------------------------------
	int radius = 2;
	cv::Mat vis;

	//cv::cvtColor(imageLeft_t1, vis, cv::COLOR_GRAY2BGR, 3);
	//cv::cvtColor(imageLeft_t1, vis, cv::COLOR_BGR2GRAY);

	vis = imageLeft_t1;

	for (int i = 0; i < pointsLeft_t0.size(); i++)
	{
		cv::circle(vis, cv::Point(pointsLeft_t0[i].x, pointsLeft_t0[i].y), radius, CV_RGB(0, 255, 0));
	}

	for (int i = 0; i < pointsLeft_t1.size(); i++)
	{
		cv::circle(vis, cv::Point(pointsLeft_t1[i].x, pointsLeft_t1[i].y), radius, CV_RGB(255, 0, 0));
	}

	for (int i = 0; i < pointsLeft_t1.size(); i++)
	{
		cv::line(vis, pointsLeft_t0[i], pointsLeft_t1[i], CV_RGB(0, 255, 0));
	}

	cv::imshow("vis", vis);
	cv::waitKey(500);

}



void trackingFrame2Frame(cv::Mat& projMatrl, cv::Mat& projMatrr,
	std::vector<cv::Point2f>& pointsLeft_t1,
	cv::Mat& points3D_t0,
	cv::Mat& rotation, cv::Mat& translation, bool mono_rotation )
{
	// Calculate frame to frame transformation
	cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);

	cv::Mat intrinsic_matrix = (cv::Mat_<float>(3, 3) << projMatrl.at<float>(0, 0), projMatrl.at<float>(0, 1), projMatrl.at<float>(0, 2),
		projMatrl.at<float>(1, 0), projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2),
		projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2), projMatrl.at<float>(1, 3));

	int iterationsCount = 500;         // number of Ransac iterations.
	float reprojectionError = .5;      // maximum allowed distance to consider it an inlier.
	float confidence = 0.999;          // RANSAC successful confidence.
	bool useExtrinsicGuess = true;
	int flags = cv::SOLVEPNP_ITERATIVE;

	cv::Mat inliers;
	cv::solvePnPRansac(points3D_t0, pointsLeft_t1, intrinsic_matrix, distCoeffs, rvec, translation,useExtrinsicGuess, iterationsCount, reprojectionError, confidence,inliers, flags);
	cv::Rodrigues(rvec, rotation);
	std::cout << "[trackingFrame2Frame] inliers size: " << inliers.size() << " out of " << pointsLeft_t1.size() << std::endl;

}