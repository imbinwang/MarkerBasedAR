#ifndef _UTILITY_H_
#define _UTILITY_H_

#include "CameraCalibration.h"
#include "GeometryTypes.h"
#include "glm.h"
#include <vector>
#include <opencv2/opencv.hpp>

float perimeter(const std::vector<cv::Point2f> &a);

bool isInto(cv::Mat &contour, std::vector<cv::Point2f> &b);

void projectPoints(const std::vector<cv::Point3f> &points3d, const CameraCalibration &camCalib,
	const Transformation &trans, std::vector<cv::Point2f> &points2d);

void calculateObjTransformation(const std::vector<Transformation> &markerTransformation, 
	float *marker2objTranslation, std::vector<Transformation> &objTransformation);

void drawCoordinates(const CameraCalibration &camCalib,
	const std::vector<Transformation> &trans, cv::Mat &img);

void drawObject( const CameraCalibration &camCalib,const std::vector<Transformation> &trans,
	GLMmodel *objModel, cv::Mat &img);

#endif
