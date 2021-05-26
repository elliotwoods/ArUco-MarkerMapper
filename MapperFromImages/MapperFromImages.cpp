#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco.h>
#include "debug.h"
#include "markermapper.h"
#include "sglviewer.h"
#include "win_dirent.h"

using namespace std;

class CmdLineParser {
int argc; char** argv; public:
    CmdLineParser(int _argc, char** _argv) :argc(_argc), argv(_argv) {}
    bool operator[] (string param) { int idx = -1;  for (int i = 0; i < argc && idx == -1; i++) if (string(argv[i]) == param) idx = i;    return (idx != -1); }
    string operator()(string param, string defvalue = "-1") { int idx = -1;    for (int i = 0; i < argc && idx == -1; i++) if (string(argv[i]) == param) idx = i; if (idx == -1) return defvalue;   else  return (argv[idx + 1]); }
};

vector<string> readDir(string path) {
    DIR* dir;
    struct dirent* ent;
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


int main(int argc, char** argv) {
    try {

        CmdLineParser cml(argc, argv);
        if (argc < 6 || cml["-h"]) {
            cerr << "Usage:  directoryWithImages camera_parameters.yml   marker_size(meters)  dictionary out  [-ref id]  [-c arucoConfig.yml]  " << endl;
            cerr << "\tDictionaries: ";
            for (auto dict : aruco::Dictionary::getDicTypes())
                cerr << dict << " ";
            cerr << endl;
            return -1;
        }




        aruco::CameraParameters Camera;
        Camera.readFromXMLFile(argv[2]);
        float markerSize = atof(argv[3]);
        string dict = argv[4];
        string outBaseName = argv[5];

        int ref_Marker_Id = -1;
        if (argc >= 7)
            if (string(argv[6]) == "-ref") {
                ref_Marker_Id = stoi(argv[7]);
            }
        //aruco_mm::debug::Debug::setLevel( stoi(cml("-debug","10")));
         //start processing
        auto AMM = aruco_mm::MarkerMapper::create();
        AMM->setParams(Camera, markerSize, ref_Marker_Id);
        if (cml["-c"])
            AMM->getMarkerDetector().loadParamsFromFile(cml("-c"));
        else
            AMM->getMarkerDetector().setDictionary(dict);
        //    AMM->getMarkerDetector().setDetectionMode(aruco::DM_FAST,0.02);
        char key = 0;
        cv::Mat image, image2;
        vector<string> files = readDir(argv[1]);

        //        std::rotate(files.begin(),files.begin()+10,files.end());
        //rotate vector
        int frameidx = 0;

        for (size_t i = 0; i < files.size() && key != 27; i++) {


            cerr << "Reading..." << files[i] << endl;
            if (files[i].back() == '.') continue;//skip . and ..
            image = cv::imread(files[i]);
            if (image.empty())continue;

            if (image.rows == Camera.CamSize.width && image.cols == Camera.CamSize.height) {//auto orient by rotation
                cv::Mat aux;
                cv::transpose(image, aux);
                cv::flip(aux, image, 0);
            }

            if (image.rows != Camera.CamSize.height || image.cols != Camera.CamSize.width) {
                cerr << "wont process THIS image because is not of the dimensions of the camera calibration file provided" << endl;
                continue;
            }

            if (image.channels() == 3) {
                cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
            }
            AMM->process(image
                , frameidx++
                , files[i]
                , Camera.CameraMatrix
                , Camera.Distorsion);
            AMM->drawDetectedMarkers(image, 3);
            cv::resize(image, image2, cv::Size(1280, 960));
            cv::imshow("image", image2);
            key = cv::waitKey(10);
        }

        //finish processing

        AMM->optimize();
        //        AMM->saveToFile(outBaseName+".amm");
        AMM->getFrameSet().saveToFile("fset.fset");
        AMM->saveToPcd(outBaseName + ".pcd", true);
        AMM->saveFrameSetPosesToFile(outBaseName + ".log");
        AMM->getCameraParams().saveToFile(outBaseName + "-cam.yml");
        AMM->getMarkerMap().saveToFile(outBaseName + ".yml");

        OpenCvMapperViewer Viewer;
        aruco::MarkerMap mmap;
        mmap.readFromFile(outBaseName + ".yml");
        Viewer.setParams(mmap, 1.5, 1280, 960, "map_viewer");
        key = 0;
        while (key != 27)
            key = Viewer.show();


    }
    catch (std::exception& ex) {
        cerr << ex.what() << endl;
    }



}
