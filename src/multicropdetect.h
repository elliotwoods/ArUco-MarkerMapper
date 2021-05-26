#pragma once

#include <opencv2/opencv.hpp>
#include <aruco.h>
#include <vector>

cv::Mat drawMarkers(cv::Mat image, const std::vector<aruco::Marker> & markers, std::string text = "");
std::vector<aruco::Marker> findMarkersMultiCrop(aruco::MarkerDetector & markerDetector, const cv::Mat & image, int cropIterations = 1, float overlap = 0.2f);
