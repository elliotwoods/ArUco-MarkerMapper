#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/cameraparameters.h>
#include "markermapper.h"

#include <random>
#include <opencv2/calib3d/calib3d.hpp>
#include "markermapper.h"
#include "utils/utils3d.h"
#include <opencv2/imgproc.hpp>
using namespace std;


int main(int argc,char **argv){

    try{
    if(argc!=3){cerr<<"uSAGe: mapper video"<<endl;return -1;}
    auto gmm=aruco_mm::MarkerMapper::readFromFile(argv[1]);


    //check the frames and print info
    for(auto frame:gmm->getFrameSet()){
        cout<<"frame idx="<<frame.first<<" vsible markers"<<endl;
        for(auto m:frame.second.markers) cout<<m.id<<" " ;
        cout<<endl;
    }

    gmm->saveToPcd("poses.pcd");

    cv::VideoCapture video;
    video.open(argv[2]);
    if (!video.isOpened()) throw std::runtime_error("Could not open video");

    //go to first valid frame and project its markers in the image

    aruco_mm::FrameInfo first_valid_frame=gmm->getFrameSet().begin()->second;
    cout<<"first valid frame is:"<<first_valid_frame.frame_idx<<endl;
    int cur_idx=0;
    video.grab();
    while(cur_idx++!=first_valid_frame.frame_idx)video.grab();
    cv::Mat image;
    video.retrieve(image);
    //get the markers and project in the image

    vector<cv::Point3f> all_points;
    for(auto marker:first_valid_frame.markers){
        aruco_mm::MarkerInfo mi=gmm->getMarkerSet()[ marker.id];
        //move its points to global ref system and add to the vector
        auto m_p3d=mi.get3dPoints();//this function returns the points in global coordinates already
        all_points.insert(all_points.end(),m_p3d.begin(),m_p3d.end());
    }
    //now, project in this camera ref system
    //first, divide r and t
    vector<cv::Point2f> points2d;
    cv::Mat r,t;
    aruco_mm::getRTfromMatrix44(first_valid_frame.rt_c2g,r,t);
    cv::projectPoints(all_points,r,t,gmm->getCameraParams().CameraMatrix,gmm->getCameraParams().Distorsion,points2d);

    //now, draw
    for(auto p:points2d)
        cv::rectangle(image,p-cv::Point2f(2,2),p+cv::Point2f(2,2), cv::Scalar(0,255,0),-1 );
    cv::imshow("image",image);
    //wait for key esc
    char k=0;
    while(k!=27) k=cv::waitKey(0);



    }catch(std::exception &ex){
              cerr<<ex.what()<<endl;
    }



}

