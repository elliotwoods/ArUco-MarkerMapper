#include "multicropdetect.h"

using namespace std;

cv::Mat drawMarkers(cv::Mat image, const vector<aruco::Marker> & markers, string text) {
	//draw outlines on the found markers
	vector<vector<cv::Point2i>> lines;
	for (const auto & marker : markers) {
		vector<cv::Point2i> points;
		for (const auto & point : marker) {
			points.push_back(point);
		}
		lines.push_back(points);
	}

	cv::Mat imageWithMarkings;
	cv::cvtColor(image, imageWithMarkings, cv::COLOR_GRAY2RGB);
	cv::polylines(imageWithMarkings, lines, true, cv::Scalar(255, 0, 0, 255), 2);

	if (!text.empty()) {
		putText(imageWithMarkings
			, text
			, cv::Point(50, 200)
			, cv::FONT_HERSHEY_SIMPLEX
			, 4
			, cv::Scalar(0, 255, 255)
			, 4);
	}

	return imageWithMarkings;
}

//#define CROPDEBUG

vector<aruco::Marker> findMarkersMultiCrop(aruco::MarkerDetector & markerDetector, const cv::Mat & image, int cropIterations, float overlap) {
	auto imageWidth = image.cols;
	auto imageHeight = image.rows;

	map<int, aruco::Marker> markersInAllCrops;

	for (int cropIteration = 0; cropIteration < cropIterations; cropIteration++) {
		auto stepRatio = 1.0f / pow(2, cropIteration);
#ifdef CROPDEBUG
		cout << cropIteration << ", " << stepRatio << endl;
#endif

		int stepWidth = imageWidth * stepRatio;
		int stepHeight = imageHeight * stepRatio;

		if (stepWidth == 0 || stepHeight == 0) {
#ifdef CROPDEBUG
			cout << "Skipping crop tier" << endl;
#endif
			continue;
		}

		int overlapWidth = stepWidth * overlap;
		int overlapHeight = stepHeight * overlap;

		for (int x = 0; x <= imageWidth - stepWidth; x += stepWidth) {
			for (int y = 0; y <= imageHeight - stepHeight; y += stepHeight) {

				//calc clamped window within image
				int x_clamped = max(x - overlapWidth, 0);
				int y_clamped = max(y - overlapHeight, 0);

				int width_clamped = stepWidth + overlapWidth;
				int height_clamped = stepHeight + overlapHeight;

				width_clamped = min(width_clamped, imageWidth - x);
				height_clamped = min(height_clamped, imageHeight - y);

				//check we have an image
				if (width_clamped == 0 || height_clamped == 0) {
#ifdef CROPDEBUG
					cout << "Skipping crop section" << endl;
#endif
					continue;
				}

				cv::Rect roi(x_clamped, y_clamped, width_clamped, height_clamped);
#ifdef CROPDEBUG
				cout << roi << "[" << imageWidth << "x" << imageHeight << "]" << endl;
#endif

				cv::Mat cropped = image(roi);

				//perform detection
				auto markersInCrop = markerDetector.detect(cropped);

#ifdef CROPDEBUG
				//preview result
				cv::imshow("cropped", drawMarkers(cropped, markersInCrop));
				cv::waitKey(0);
#endif

				//translate into image coords
				for (auto & markerInCrop : markersInCrop) {
					for (auto & point : markerInCrop) {
						point.x += roi.x;
						point.y += roi.y;
					}
				}

				//save into markers
				for (const auto & marker : markersInCrop) {
					//this overwrites any previous finds
					markersInAllCrops[marker.id] = marker;
				}
			}
		}
	}

	//assemble vector of markers
	vector<aruco::Marker> markerVector;
	for (const auto & markerIt : markersInAllCrops) {
		markerVector.push_back(markerIt.second);
	}

	if (true) {
		//subpix per marker
		for (auto & marker : markerVector) {
			cout << marker.id << ", ";
			auto delta = marker[1] - marker[0];
			auto length2 = delta.dot(delta);
			auto length = sqrt(length2);
			cout << " delta=" << delta;
			cout << " length=" << length;

			auto lengthPart = length / 8; // a square
			lengthPart /= 1; // portion of square

			lengthPart /= 2;
			lengthPart += 1;

			auto winSize = cv::Size(lengthPart, lengthPart);
			cout << "winSize" << winSize << endl;

			//perform subpix
			cv::cornerSubPix(image
				, marker
				, winSize
				, cv::Size(-1, -1)
				, cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 50, 1e-6));
		}
	}

	return markerVector;
}