#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include "utils/se3.h"
#include <map>
using namespace std;
using namespace aruco_mm;
std::map<uint32_t,aruco_mm::se3> flog1,flog2;




cv::Mat  getMatrix(double tx,double ty ,double tz,double qx,double qy, double qz,double qw){

     double qx2 = qx*qx;
    double qy2 = qy*qy;
    double qz2 = qz*qz;


    cv::Mat m=cv::Mat::eye(4,4,CV_32F);

    m.at<float>(0,0)=1 - 2*qy2 - 2*qz2;
    m.at<float>(0,1)=2*qx*qy - 2*qz*qw;
    m.at<float>(0,2)=2*qx*qz + 2*qy*qw;
    m.at<float>(0,3)=tx;

    m.at<float>(1,0)=2*qx*qy + 2*qz*qw;
    m.at<float>(1,1)=1 - 2*qx2 - 2*qz2;
    m.at<float>(1,2)=2*qy*qz - 2*qx*qw;
    m.at<float>(1,3)=ty;

    m.at<float>(2,0)=2*qx*qz - 2*qy*qw	;
    m.at<float>(2,1)=2*qy*qz + 2*qx*qw	;
    m.at<float>(2,2)=1 - 2*qx2 - 2*qy2;
    m.at<float>(2,3)=tz;
    return m;
}

std::vector<cv::Vec4f> getLine(const  cv::Point3f &a,const  cv::Point3f &b,cv::Scalar color,int npoints){
    float fcolor;uchar *c=(uchar*)&fcolor;
    for(int i=0;i<3;i++)c[i]=color[i];

    std::vector<cv::Vec4f>  points;
    cv::Vec4f pa(a.x,a.y,a.z,fcolor);
    cv::Vec4f pb(b.x,b.y,b.z,fcolor);
    cv::Vec4f d =pb-pa;
    d*=1./cv::norm(d);
    double step=  cv::norm(pb-pa)*( 1./ double(npoints));
    //cout<<"step="<<step<<endl;
    for(int i=0;i<npoints;i++){
        points.push_back(pa+ (d*step*double(i)));
    }
  //  cout<<pa<<" "<<pb<<" "<<pa+ (d*step*double(npoints))<<endl;
    return points;

}

std::map<uint32_t,aruco_mm::se3> loadFile(std::string fp)throw(std::exception){
    std::map<uint32_t,aruco_mm::se3> fmap;
    ifstream file(fp);
    if (!file)throw std::runtime_error("Could not open file");
    uint32_t stamp;
    float tx,ty,tz,qx,qy,qz,qw;
    while(!file.eof()){
        if (file>>stamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw)
            fmap.insert(make_pair (stamp, se3(getMatrix(tx,ty,tz,qx,qy,qz,qw))));
    }
    return fmap;
}


cv::Vec4f convert(cv::Point3f p,cv::Scalar color){

    float fcolor;uchar *c=(uchar*)&fcolor;

    for(int i=0;i<3;i++)c[i]=color[i];
    return cv::Vec4f(p.x,p.y,p.z,fcolor);
}
int main(int argc,char **argv){

    if(argc!=3){cerr<<"logfile1_grounftruth   out.pcd"<<endl;return -1;}
    vector<cv::Vec4f> pcdpoints;
    auto file=loadFile(argv[1]);


    std::pair<uint32_t,aruco_mm::se3> prev;
    int idx=0;
    for(auto pose:loadFile(argv[1])){
        if (idx!=0){
            auto points= getLine(cv::Point3f(prev.second.rt[3],prev.second.rt[4],prev.second.rt[5])
                                ,cv::Point3f(pose.second.rt[3],pose.second.rt[4],pose.second.rt[5])
                                ,cv::Scalar(0,0,0),100);
            pcdpoints.insert(pcdpoints.end(),points.begin(),points.end());
        }
        prev=pose;
        idx++;
    }

    std::ofstream filePCD ( argv[2], std::ios::binary );
    filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<pcdpoints.size()<<"\nHEIGHT 1\nPOINTS "<<pcdpoints.size()<<"\nDATA binary\n";
    filePCD.write((char*)&pcdpoints[0],pcdpoints.size()*sizeof(pcdpoints[0]));

}

