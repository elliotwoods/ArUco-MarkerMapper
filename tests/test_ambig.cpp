
#include "optimizers/ippe.h"
#include "markermapper.h"
#include "utils/utils3d.h"
#include <opencv2/highgui/highgui.hpp>
#include "debug.h"
int main(int argc,char **argv){

//    try{
//        if (argc!=2){cerr<<"Usage:     mmaper "<<endl;return -1;}
//        cerr<<"go"<<endl;
//        auto mapper=aruco_mm::MarkerMapper::readFromFile(argv[1]);
//        cout<<mapper->getMarkerSet().size()<<endl;
//        auto mpoints=aruco_mm::getMarkerPoints( mapper->getDefaultMarkerSize());
//        auto & cam_params=mapper->getCameraParams();
//        for(auto frame:mapper->getFrameSet()){
//            cout<<"frame:"<<frame.first<<endl;
//            for(auto m1:frame.second.markers){
//                auto poses= IPPE::solvePnP_(mpoints,m1,cam_params.CameraMatrix,cam_params.Distorsion);
//                cout<<"--="<<aruco_mm::getReprjError(mpoints,m1,cam_params.CameraMatrix,cam_params.Distorsion,frame.second.rt_c2g *mapper->getMarkerSet()[ m1.id].rt_g2m )<<endl;

//                cout<<"a="<<aruco_mm::getReprjError(mpoints,m1,cam_params.CameraMatrix,cam_params.Distorsion,poses[0].first)<<endl;
//                cout<<"b="<<aruco_mm::getReprjError(mpoints,m1,cam_params.CameraMatrix,cam_params.Distorsion,poses[1].first)<<endl;
//                for(auto m2:frame.second.markers){
//                    if(m1.id!=m2.id){
//                    cout<<"m1:"<<m1.id<<" "<<m2.id<<endl;
//                    assert(mapper->getMarkerSet().find(m.id)!=mapper->getMarkerSet().end());
//                    cout<<mapper->getMarkerSet()[ m1.id]<<endl;
//                    cv::Mat m12=mapper->getMarkerSet()[ m1.id].rt_g2m.inv() *mapper->getMarkerSet()[ m2.id].rt_g2m;
//                    cout<<m12<<endl;
//                    vector<cv::Point3f> mpoints2;
//                    for(auto p:mpoints){cerr<<p<<" "; mpoints2.push_back(aruco_mm::mult<float>( m12,p));cout<<mpoints2.back()<<endl;}
//                    cout<<"rp1="<<aruco_mm::getReprjError(mpoints2,m2,cam_params.CameraMatrix,cam_params.Distorsion,poses[0].first)<<endl;
//                    cout<<"rp2="<<aruco_mm::getReprjError(mpoints2,m2,cam_params.CameraMatrix,cam_params.Distorsion,poses[1].first)<<endl;

//                    cout<<"....................."<<endl;

//                    cin.ignore();
//                    }
//                }
//            }
//            cin.ignore();
//        }

//    }catch(std::exception &ex){
//        cerr<<ex.what()<<endl;
//    }
}

