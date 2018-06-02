#include <aruco/markermap.h>
#include "markermapper.h"
#include "utils/utils3d.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;

void   getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,float &qx,float &qy,float &qz,float &qw,float &tx,float &ty,float &tz){
    //get the 3d part of matrix and get quaternion
    assert(M_in.total()==16);
 //   cout<<"=="<<M_in<<endl;
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
int main(int argc,char **argv){

    try{
        if (argc!=4){cerr<<"Usage: in.amm board   out.log"<<endl;return -1;}
        aruco::MarkerMap bc;
        auto Mmapper=aruco_mm::MarkerMapper::readFromFile(argv[1]);
        bc.readFromFile(argv[2]);
        //
        cout<<"msize="<<Mmapper->getDefaultMarkerSize() <<endl;
        auto bc_meters=bc.convertToMeters(Mmapper->getDefaultMarkerSize() );

        //now, do horn transform
        vector<cv::Point3f> pboard,pmap;
        for(auto mi: bc_meters){
            //find
            if ( Mmapper->getMarkerSet().find(mi.id )!=Mmapper->getMarkerSet().end()){
                cout<<"i="<<mi.id<<endl;
                pboard.insert(pboard.end(),mi.points.begin(),mi.points.end() );
                auto mi2=Mmapper->getMarkerSet()[mi.id].get3dPoints();
                pmap.insert(pmap.end(),mi2.begin(),mi2.end());
            }
            else cerr<<"mi.id "<<mi.id<<" not found"<<endl;
        }
        cv::Mat r,t;
        cout<<"err="<<aruco_mm::rigidBodyTransformation_Horn1987(pboard,pmap,r,t)<<endl;
        auto m44_g=aruco_mm::getRTMatrix(r,t,CV_32F);
cerr<<"lklkj"<<endl;

        //obtain the rmse
        {
        double err=0;
         vector<cv::Point3f> pboard_t;
         for(auto p:pboard)pboard_t.push_back(aruco_mm::mult(m44_g,p));
        for(int i=0;i<pboard_t.size();i++)
            err+=cv::norm(pboard_t[i]-pmap[i])*cv::norm(pboard_t[i]-pmap[i]);
        err/=double(pboard_t.size());
        cerr<<"eerr mean ="<<err<<endl;
        err=sqrt(err);
        cerr<<"rmse="<<err<<endl;
    }
        //now move all points to the
        for(auto &mi:bc_meters)
            for(auto &p:mi.points) p=aruco_mm::mult(m44_g,p);

        //now, create the log for comparison of trayectories
        ofstream file(argv[3]);
        if (!file) throw std::runtime_error("Could not open log file");
        for(auto f:Mmapper->getFrameSet()){
            vector<cv::Point3f> p3d;
            vector<cv::Point2f> p2d;
            for(auto m:f.second.markers){
                auto pp=bc_meters.getMarker3DInfo(m.id);
                p3d.insert(p3d.end(),pp.points.begin(),pp.points.end());
                p2d.insert(p2d.end(),m.begin(),m.end());
            }
            cv::Mat rvec,tvec;
            cv::solvePnPRansac(p3d,p2d,Mmapper->getCameraParams().CameraMatrix,Mmapper->getCameraParams().Distorsion,rvec,tvec);

            //now, get the quaternions

            cv::Mat m4x4=aruco_mm::getRTMatrix(rvec,tvec,CV_32F).inv();
         //   cout<<m4x4<<endl;
         //   cout<<f.second.rt_c2g<<endl;

            float tx,ty,tz,qx,qy,qz,qw;
            getQuaternionAndTranslationfromMatrix44(m4x4 , qx, qy,qz,qw,tx,ty,tz);
            file<<f.first<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;
          //  cout<<f.first<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;
//        cin.ignore();

        }
    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }

}
