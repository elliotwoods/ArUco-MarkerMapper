#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv_modules.hpp>
#include <aruco.h>
#include "debug.h"
#include "markermapper.h"
#include "sglviewer.h"
#if WIN32
#include "win_dirent.h"
#else
#include <dirent.h>
#endif
using namespace std;

class CmdLineParser {
int argc; char **argv; public:
	CmdLineParser(int _argc, char **_argv) :argc(_argc), argv(_argv) {}
	bool operator[] (string param) { int idx = -1;  for (int i = 0; i < argc && idx == -1; i++) if (string(argv[i]) == param) idx = i;    return (idx != -1); }
	string operator()(string param, string defvalue = "-1") { int idx = -1;    for (int i = 0; i < argc && idx == -1; i++) if (string(argv[i]) == param) idx = i; if (idx == -1) return defvalue;   else  return (argv[idx + 1]); }
};

vector<string> readDir(string path) {
	DIR *dir;
	struct dirent *ent;
	vector<string>  res;
	if ((dir = opendir(path.c_str())) != NULL) {
		/* print all the files and directories within directory */
		while ((ent = readdir(dir)) != NULL)
			res.push_back(path + string("/") + string(ent->d_name));
		closedir(dir);
	}
	//check
	return res;
}

cv::Mat drawMarkers(cv::Mat image, const vector<aruco::Marker> & markers) {
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

	return imageWithMarkings;
}

//#define CROPDEBUG

vector<aruco::Marker> findMarkersMultiCrop(aruco::MarkerDetector & markerDetector, const cv::Mat & image, int cropIterations = 1, float overlap = 0.2f) {
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

	//subpix per marker
	for (auto & marker : markerVector) {
		cout << marker.id << ", ";
		auto delta = marker[2] - marker[1];
		auto length2 = delta.dot(delta);
		auto length = sqrt(length2);
		auto lengthPart = length / 16; // half a square

		lengthPart /= 2;
		lengthPart += 1;

		auto winSize = cv::Size(lengthPart, lengthPart);

		//perform subpix
		cv::cornerSubPix(image
			, marker
			, winSize
			, cv::Size(-1, -1)
			, cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 50, 1e-6));
	}

	return markerVector;
}

int main(int argc, char **argv) {
	try {

		CmdLineParser cml(argc, argv);
		if (argc < 2 || cml["-h"]) {
			cerr << "Usage:  directoryWithImages" << endl;
			cerr << "\tDictionaries: ";
			for (auto dict : aruco::Dictionary::getDicTypes())
				cerr << dict << " ";
			cerr << endl;
			return -1;
		}


		string dict = "ARUCO_MIP_36h12";

		int ref_Marker_Id = -1;
		if (argc >= 7)
		{
			if (string(argv[6]) == "-ref") {
				ref_Marker_Id = stoi(argv[7]);
			}
		}

		aruco::MarkerDetector detector;

		//set detector parameters
		if (cml["-c"]) {
			detector.loadParamsFromFile(cml("-c"));
		}
		else {
			detector.setDictionary(dict);
			auto params = detector.getParameters();
			{
				//actually detection is much worse with this turned on
				//params.enclosedMarker = true;
				params.setDetectionMode(aruco::DetectionMode::DM_NORMAL, 0.0f);
				//params.detectEnclosedMarkers(true);

			}
			detector.setParameters(params); // not sure if we'd need to do this if we used a reference
		}

		char key = 0;
		cv::Mat image, image2;
		vector<string> files = readDir(argv[1]);

		int frameidx = 0;

		for (size_t i = 0; i < files.size() && key != 27; i++) {
			cerr << "Reading..." << files[i] << endl;
			if (files[i].back() == '.') continue;//skip . and ..
			image = cv::imread(files[i]);
			if (image.empty()) {
				cout << "No image" << endl;
			}

			//HACK THIS SEEMS TO DANGEROUS TO KEEP
			//            if (image.rows==Camera.CamSize.width && image.cols==Camera.CamSize.height ){//auto orient by rotation
			//                cv::Mat aux;
			//                cv::transpose(image,aux);
			//                cv::flip(aux,image,0);
			//            }

			if (image.channels() != 1) {
				cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
			}

			auto markers = findMarkersMultiCrop(detector
				, image
				, 4
				, 0.5f);


			auto imageWithMarkings = drawMarkers(image, markers);

			cv::resize(imageWithMarkings, image2, cv::Size(1920, 1920 * image.rows / image.cols));

			cv::imshow("image", image2);
			cv::waitKey(0);
		}
	}
	catch (std::exception &ex) {
		cerr << ex.what() << endl;
	}
}
