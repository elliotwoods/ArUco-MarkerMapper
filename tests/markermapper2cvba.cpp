#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/cameraparameters.h>
#include "markermapper.h"
#include <random>
#include <cvba/bundler.h>
#include "utils/se3.h"

#include "utils/utils3d.h"
using namespace std;

using namespace aruco_mm;

void toSba(    shared_ptr<aruco_mm::MarkerMapper> mm, cvba::BundlerData<double> &bdata){
    bdata.resize(mm->getFrameSet().size(),mm->getMarkerSet().size()*4);
    //set the points
    std::map<uint32_t,uint32_t> marker_pos;
    int pidx=0,midx=0;
    for(auto m:mm->getMarkerSet()){
        marker_pos.insert(make_pair(m.first,midx++));
        for(auto p:m.second.get3dPoints())
            bdata.points[pidx++]=p;
    }
    //now, the image points
    int fidx=0;
    for(auto frame:mm->getFrameSet()){
        for(auto marker:frame.second.markers){
            if (marker_pos.find(marker.id)!=marker_pos.end()){
                int moff=marker_pos[marker.id]*4;
                for(auto p2d:marker){
                    bdata.imagePoints[fidx][ moff ]=p2d;
                    bdata.visibility[fidx][moff ]=1;
                    moff++;
                }
            }
        }
        se3 pose(frame.second.rt_c2g);
        bdata.R[fidx]= pose.getRvec();
        bdata.T[fidx]= pose.getTvec();
        bdata.cameraMatrix[fidx]=mm->getCameraParams().CameraMatrix.clone();
        bdata.distCoeffs[fidx]=mm->getCameraParams().Distorsion.clone();
        fidx++;
    }
}



void fromSba(cvba::BundlerData<double> &bdata,    std::shared_ptr<aruco_mm::MarkerMapper> mm){
    bdata.resize(mm->getFrameSet().size(),mm->getMarkerSet().size()*4);
    //set the points
    std::map<uint32_t,uint32_t> marker_pos;
    int pidx=0;
    std::vector<cv::Point3f> p3d(4);
    for(auto &marker:mm->getMarkerSet()){
        for(auto &p:p3d) p=bdata.points[pidx++];
        marker.second.rt_g2m =aruco_mm::rigidBodyTransformation_Horn1987(marker.second.get3dPoints(false) ,p3d);
    }
    //now, frame locations

    int fidx=0;
    for(auto &frame:mm->getFrameSet()){
        frame.second.rt_c2g= aruco_mm::getRTMatrix( bdata.R[fidx], bdata.T[fidx]);
        fidx++;
    }
    //assume first camera is the one
    mm->getCameraParams().CameraMatrix=bdata.cameraMatrix[0];
    mm->getCameraParams().Distorsion=bdata.distCoeffs[0];

}


int main(int argc,char **argv){

    if (argc<3){cerr<<"Usage:in.amm out.ba [out.amm]"<<endl;return -1;}
    auto mm= aruco_mm::MarkerMapper::readFromFile(argv[1]);
    cvba::BundlerData<double> bdata;

    if(argc==3){
        toSba(mm,bdata);
        bdata.saveToFile(argv[2]);
    }
    else{
        bdata.readFromFile(argv[2]);
        fromSba(bdata,mm);
        mm->saveToFile( argv[3]);
        mm->saveToPcd(argv[3]+string(".pcd"));
    }

}

