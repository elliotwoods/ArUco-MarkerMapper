#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <utils/utils3d.h>
#include <utils/se3.h>
using namespace std;
std::vector<cv::Vec4f> getLine(const  cv::Point3f &a,const  cv::Point3f &b,cv::Scalar color,int npoints){
    float fcolor;uchar *c=(uchar*)&fcolor;
    for(int i=0;i<3;i++)c[i]=color[i];

    std::vector<cv::Vec4f>  points;
    cv::Vec4f pa(a.x,a.y,a.z,fcolor);
    cv::Vec4f pb(b.x,b.y,b.z,fcolor);
    cv::Vec4f d =pb-pa;

    double step= cv::norm(pa-pb) / double(npoints);
    for(int i=0;i<npoints;i++)
        points.push_back(pa+ (d*step*double(i)));
    return points;

}

float getColor(cv::Scalar color)
{
    float fcolor;uchar *c=(uchar*)&fcolor;
    for(int i=0;i<3;i++)c[i]=color[i];
    return fcolor;
}
std::vector<cv::Vec4f> getPcdPoints(const vector<cv::Point3f> &mpoints,cv::Scalar color,int npoints=100){
   vector<cv::Vec4f> points;
   double msize=cv::norm(mpoints[0]-mpoints[1]);
   float fcolor;uchar *c=(uchar*)&fcolor;
   for(int i=0;i<3;i++)c[i]=color[i];

   //lines joining points
   for(int i=0;i<mpoints.size();i++){
       cv::Point3f v=mpoints[(i+1)%mpoints.size()]-mpoints[i];
       float ax=1./float(npoints);//npoints
       for(float x=0;x<=1;x+=ax){
           cv::Point3f p3=mpoints[i]+v*x;
           points.push_back(cv::Vec4f(p3.x,p3.y,p3.z, fcolor));
       }
   }

   //line indicating direction
   //take first and second, first and last , and get the cross vector indicating the direction
   cv::Point3f v1=mpoints[1]-mpoints[0];
   cv::Point3f v2=mpoints[3]-mpoints[0];
    v1*=1./cv::norm(v1);
   v2*=1./cv::norm(v2);
   cv::Point3f vz=v2.cross(v1);
   vz*=1./cv::norm(vz);//now, unit

   //set the center
   cv::Point3f center=(mpoints[0]+mpoints[1]+mpoints[2]+mpoints[3])*0.25;
   float ax=(msize/3)/100;
   for(float x=0;x<=msize/3;x+=ax){
       cv::Point3f p3=center+vz*x;
       points.push_back(cv::Vec4f(p3.x,p3.y,p3.z, getColor(cv::Scalar(0,0,255))));

   }


//   for(float x=0;x<=2*msize/3;x+=ax){
//       cv::Point3f p3=center+v1*x;
//       points.push_back(cv::Vec4f(p3.x,p3.y,p3.z,getColor(cv::Scalar(0,255,0))));

//   }
//   for(float x=0;x<=2*msize/3;x+=ax){
//       cv::Point3f p3=center+v2*x;
//       points.push_back(cv::Vec4f(p3.x,p3.y,p3.z, getColor(cv::Scalar(255,0,0))));

//   }
   return points;

}

cv::Vec4f convert(cv::Point3f p,cv::Scalar color){

    float fcolor;uchar *c=(uchar*)&fcolor;

    for(int i=0;i<3;i++)c[i]=color[i];
    return cv::Vec4f(p.x,p.y,p.z,fcolor);
}

std::vector<cv::Vec4f>  getCube(cv::Point3f p,float size,cv::Scalar color){
    std::vector<cv::Vec4f>  ret;
    float fcolor;uchar *c=(uchar*)&fcolor;
    for(int i=0;i<3;i++)c[i]=color[i];

    float step=size/5.;
    for(double x=p.x-size;x<=p.x+size;x+=step)
        for(double y=p.y-size;y<=p.y+size;y+=step)
            for(double z=p.z-size;z<=p.z+size;z+=step)
             ret.push_back(cv::Vec4f (x,y,z,fcolor));


    return ret;
}

cv::Mat  getMatrix(double tx,double ty ,double tz,double qx,double qy, double qz,double qw){

    double qw2 = qw*qw;
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
struct pose_q{
    float q[4],t[3];
};

std::map<uint32_t,pose_q> readNMVFile(string path,vector<cv::Point3f> &points,std::vector<cv::Mat> &poses)throw(std::exception){
    std::ifstream file(path);
std::map<uint32_t,pose_q> map_ret;
    if( !file) throw std::runtime_error("COULD NOT OPEN FOILE") ;

    string sig;file>>sig;
    if(sig!="NVM_V3") throw std::runtime_error("INVALID FILE") ;

    int rotation_parameter_num = 4;
    int nposes;
    file>>nposes;
    cout<<"np="<<nposes<<endl;
    for(int i=0;i<nposes;i++){
        double f, d[2];
        string frame_name;
        cout<<i<<endl;
        file>>frame_name>>f;
        //parse frame name into number
        string number;
        for(auto c:frame_name)
            if(std::isdigit(c)) number.push_back(c);
        int inumber=std::stoi(number);
        float q[4],t[3];

        for(int j = 0; j < 4; ++j) file >> q[j];
        for(int j = 0; j < 3; ++j) file >> t[j];
        file  >> d[0] >> d[1];
        cout<<frame_name<<" " <<number<<" "<<inumber<<" "<<q[0]<<" "<<t[0]<<endl;

        double qq = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
               double qw, qx, qy, qz;
               if(qq>0)
               {
                   qw=q[0]/qq;
                   qx=q[1]/qq;
                   qy=q[2]/qq;
                   qz=q[3]/qq;
               }else
               {
                   qw = 1;
                   qx = qy = qz = 0;
                   cerr<<"NNNNNNNN"<<endl;
               }

               pose_q pq;
               pq.q[0]=qx;
               pq.q[1]=qy;
               pq.q[2]=qz;
               pq.q[3]=qw;

               for(int p=0;p<3;p++) pq.t[p]=t[p];

               map_ret.insert(std::make_pair(inumber,pq));
        poses.push_back(getMatrix(t[0],t[1],t[2],qx,qy,qz,qw) );
    }

    int number_of_points;

    file>>number_of_points;
    {
    string line;
    std::getline(file,line);
    }
    cout<<"npoints="<<number_of_points<<endl;
    for(int i=0;i<number_of_points;i++){
        string line;
        std::getline(file,line);
        if(line.size()>10){
        stringstream sstr;sstr<<line;
        cv::Point3f p;
        sstr>>p.x>>p.y>>p.z;
        points.push_back(p);
        cout<<p<<endl;
        }
    }
    return map_ret;
}
vector<cv::Point3f>   getMarkerPoints(float size,cv::Mat RT )
{
    float size_2=size/2.;
    //now, that the current location is estimated, add new markers and update old ones
    vector<cv::Point3f> points = { cv::Point3f ( -size_2, size_2,0 ),cv::Point3f ( size_2, size_2 ,0 ),
                                   cv::Point3f ( size_2, -size_2,0 ),cv::Point3f ( -size_2, -size_2,0 )  };
    if (!RT.empty())   aruco_mm::mult<cv::Point3f>(RT,points);

    return points;

}


void saveLogFile(string fpath, std::map<uint32_t,pose_q> &pose_map)
{
    ofstream file(fpath);
    if (!file){cerr<<"could not open file out log"<<endl;exit(0);}
    for(auto p:pose_map){
        file<<p.first<<" "<<p.second.t[0]<<" "<<p.second.t[1]<<" "<<p.second.t[2]<<" "<<p.second.q[0]<<" "<<p.second.q[1]<<" "<<p.second.q[2]<<" "<<p.second.q[3]<<endl;
    }
}

int main(int argc,char **argv){


    try{

        if (argc!=5){cerr<<"Usage: in.nvm out.pcd out.log scale"<<endl;return -1;}
        vector<cv::Point3f> points;
        std::vector<cv::Mat> poses;
        double scale=atof(argv[4]);
        auto map_poses=readNMVFile(argv[1],points,poses);
        //save to pcd file
        vector<cv::Vec4f> pcdpoints;
        for(auto p:points){
            pcdpoints.push_back(convert(scale*p,cv::Scalar(255,0,0)));
//            auto pp=getCube(p,0.01,cv::Scalar(0,0,255));
//            pcdpoints.insert(pcdpoints.end(),pp.begin(),pp.end());
        }
        for(auto p:poses){
//            auto pcam=getPcdPoints(getMarkerPoints(0.25,p),cv::Scalar(0,255,0));
            auto pse3=aruco_mm::se3(p);
            auto point=scale*cv::Point3f(pse3.rt[3],pse3.rt[4],pse3.rt[5]);
            pcdpoints.push_back(convert(point,cv::Scalar(0,0,255)));//(pcdpoints.end(),pcam.begin(),pcam.end());
//            auto pcam=getCube(,0.01,cv::Scalar(0,0,255));

//            pcdpoints.insert(pcdpoints.end(),pcam.begin(),pcam.end());

        }
        std::ofstream filePCD ( argv[2], std::ios::binary );
        filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<pcdpoints.size()<<"\nHEIGHT 1\nPOINTS "<<pcdpoints.size()<<"\nDATA binary\n";


        filePCD.write((char*)&pcdpoints[0],pcdpoints.size()*sizeof(pcdpoints[0]));

        saveLogFile(argv[3],map_poses);
    }catch(std::exception &ex)
    {
        cerr<<ex.what()<<endl;
    }
}
