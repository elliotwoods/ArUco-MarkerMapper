

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/cameraparameters.h>
#include "utils/utils3d.h"
#include <memory>
#include <opencv2/imgproc/imgproc.hpp>
#include "markermapper.h"
#include "debug.h"
#include <aruco/markermap.h>
#include <aruco/posetracker.h>
#include <Eigen/Geometry>
using namespace std;
class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                                    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };


void savePosesToFile(string filename,const std::map<int,cv::Mat> &fmp);
int main(int argc,char **argv){
    try{
        CmdLineParser cml(argc,argv);
        if (argc<5 || cml["-h"]){cerr<<"Usage: in.ppv  in_map.yml camera.yml out.log "<<endl;return -1;}

        aruco::MarkerMap mmap;mmap.readFromFile(argv[2]);
        aruco::MarkerMapPoseTracker Tracker;
        aruco::CameraParameters camera;
        camera.readFromXMLFile(argv[3]);
        Tracker.setParams(camera,mmap);
        std::map<int,cv::Mat> frame_pose_map;//set of poses and the frames they were detected

        aruco_mm::debug::PreprocessedVideo ppv;
        ppv.readFromFile(argv[1]);
        cout<<"NFRAMES="<<ppv._frameSet.size()<<endl;
        cout<<"markersize="<<ppv._markerSize<<endl;
        for(auto &frame:ppv._frameSet){
            if ( Tracker.estimatePose(frame.second.markers)){
                frame_pose_map.insert(make_pair(frame.first,Tracker.getRTMatrix()));
                frame.second.rt_c2g=Tracker.getRTMatrix();
            }
        }


        savePosesToFile(argv[4],frame_pose_map);
        aruco_mm::MarkerSet ms;
        ms.insert(make_pair(0,aruco_mm::MarkerInfo(0,cv::Mat::eye(4,4,CV_32F),0.1)));
        aruco_mm::savePCDFile(argv[4]+string(".pcd"),ms,ppv._frameSet,0,false,cv::Scalar(255,0,0),cv::Scalar(100,100,255));
    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }

}

void  getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,float &qx,float &qy,float &qz,float &qw,float &tx,float &ty,float &tz){
    //get the 3d part of matrix and get quaternion
    assert(M_in.total()==16);
    cv::Mat M;M_in.convertTo(M,CV_32F);
    cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
    //use now eigen
    Eigen::Matrix3f e_r33;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            e_r33(i,j)=M.at<float>(i,j);

    //now, move to a angle axis
    Eigen::Quaternionf q(e_r33);
    qx=q.x();
    qy=q.y();
    qz=q.z();
    qw=q.w();


    tx=M.at<float>(0,3);
    ty=M.at<float>(1,3);
    tz=M.at<float>(2,3);


}
void savePosesToFile(string filename,const std::map<int,cv::Mat> &fmp){
    std::ofstream file(filename);
    float qx,qy,qz,qw,tx,ty,tz;
    for(auto frame:fmp){
        if ( !frame.second.empty()){
                getQuaternionAndTranslationfromMatrix44(frame.second,qx,qy,qz,qw,tx,ty,tz);
                file<<frame.first<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;
        }
    }
}
