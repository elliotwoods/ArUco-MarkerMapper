#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include "utils/se3.h"
#include "markermapper.h"
#include <map>
#include <Eigen/Geometry>
using namespace std;
using namespace aruco_mm;
std::map<uint32_t,aruco_mm::se3> flog1,flog2;

void  getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,float &qx,float &qy,float &qz,float &qw,float &tx,float &ty,float &tz);
cv::Mat  getMatrix(double tx,double ty ,double tz,double qx,double qy, double qz,double qw);



std::map<uint32_t,se3> loadFile(std::string fp,bool invert=false){
    std::map<uint32_t,se3> fmap;
    ifstream file(fp);
    float stamp;
    float tx,ty,tz,qx,qy,qz,qw;
    while(!file.eof()){
        string line;
        std::getline(file,line);

        stringstream sline;sline<<line;
         if (sline>>stamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw){
             auto m=getMatrix(tx,ty,tz,qx,qy,qz,qw);
             if (invert)m=m.inv();
            fmap.insert(make_pair (stamp, se3(m)));
        }
     }
    return fmap;
}

cv::Vec4f convert(cv::Point3f p,cv::Scalar color){

    float fcolor;uchar *c=(uchar*)&fcolor;

    for(int i=0;i<3;i++)c[i]=color[i];
    return cv::Vec4f(p.x,p.y,p.z,fcolor);
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

class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                                    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };

int main(int argc,char **argv){
    CmdLineParser cml(argc,argv);

    if(argc<3 || cml["-h"]){cerr<<"logfile1_grounftruth logfile2 [-out_pcd out.pcd] [-out_pcd_gt out.pcd] [-out_pcd_est out.pcd] [-smin val] [-smax val] [-flip_y 0:gt 0:est] [-gt_scaled_translated_log file] [-est_color <r,g,b>]  [-gt_color <r,g,b>] [-conv 0:gt->est 1:est->gt] [-mm <in:out.pcd> convert markermap to the ground truth] "<<endl;return -1;}

    auto f_est=loadFile(argv[2]);
    auto f_gt=loadFile(argv[1]);
    double smin=std::stof(cml("-smin","0.01"));
    double smax=std::stof(cml("-smax","1.8"));
cout<<smin<<" "<<smax<<endl;
    double min_err=std::numeric_limits<double>::max();
    cv::Mat best_T;
    double scale_best=-1;
    double flip_gt_y=1;
    if (cml["-flip_y"])
        if (stoi( cml("-flip_y","0"))==0)
            for(auto &frame:f_gt)frame.second.rt[4]*=-1;
    else
            for(auto &frame:f_est)frame.second.rt[4]*=-1;


    vector<std::pair<cv::Point3f,cv::Point3f> > matches;
    for(auto frame_est:f_est){
        auto frame_gt=f_gt.find( frame_est.first);
        if (frame_gt!=f_gt.end())
            matches.push_back(make_pair(cv::Point3f(frame_gt->second.rt[3],frame_gt->second.rt[4],frame_gt->second.rt[5]),cv::Point3f( frame_est.second.rt[3],frame_est.second.rt[4],frame_est.second.rt[5])));
             }


    for(float sc=smin;sc<=smax;sc+=0.001){
        //get the best possible correspondece between reference systems
        vector<cv::Point3f> points_gt,points_est;
        for(auto m:matches){
            points_gt.push_back( m.first);
            points_est.push_back(sc*m.second);
        }
        //now, do horn to match
        cv::Mat r,t;
        double err=aruco_mm::rigidBodyTransformation_Horn1987(points_est,points_gt,r,t);



          cout<<"scale="<<sc<<" "<<err<<endl;
        if (min_err>err){
            scale_best=sc;
            min_err=err;
            best_T=aruco_mm::getRTMatrix(r,t);
        }
    }
    cerr<<"Best scale="<<scale_best<<" err="<<min_err<<endl;
cout<<"...."<<endl;
    cv::Mat scaleMatrix=cv::Mat::eye(4,4,CV_32F);
    scaleMatrix.at<float>(0,0)=scaleMatrix.at<float>(1,1)=scaleMatrix.at<float>(2,2)=scale_best;
    cout<<"....1"<<endl;

    best_T=best_T*scaleMatrix;

    cout<<"....2:"<<best_T<<endl;

    //also, make both initial points the same
    auto p0_res=aruco_mm::mult(best_T,matches[0].second);
    cout<<"....3"<<endl;

    cv::Point3f dif= matches[0].first-p0_res;
    cv::Mat t_m=cv::Mat::eye(4,4,CV_32F);
    t_m.at<float>(0,3)=dif.x;
    t_m.at<float>(1,3)=dif.y;
    t_m.at<float>(2,3)=dif.z;
    best_T=t_m*best_T;

    cout<<"...."<<endl;
     //create points
    vector<cv::Point3f> points_gt,points_est;

    for(auto m:matches){
        points_gt.push_back( m.first);
        points_est.push_back( aruco_mm::mult(best_T,m.second) );
     }


     cout<<"...."<<endl;



    if (cml["-out_pcd"] || cml["-out_pcd_gt"]|| cml["-out_pcd_est"]){

        cv::Scalar color_est_points(0,0,255),color_gt_points(0,0,0);
        auto read_color=[](string str){
            cv::Scalar  color;
            for(auto &c:str) if(c==',')c=' ';
             sscanf(str.c_str(),"%lf %lf %lf",&color[0],&color[1],&color[2]);
            return color;
        };

        if(cml["-est_color"]) color_est_points=read_color(cml("-est_color"));
        if(cml["-gt_color"]) color_gt_points=read_color(cml("-gt_color"));

        vector<cv::Vec4f> pcdpoints_gt,pcdpoints_est;
        //transfor points



        for(int i=1;i<points_gt.size();i++)        {
            auto pp=getLine(points_gt[i-1],points_gt[i],color_gt_points,100);
             pcdpoints_gt.insert(pcdpoints_gt.end(),pp.begin(),pp.end());
        }

        for(int i=1;i<points_est.size();i++)        {
            auto pp=getLine(points_est[i-1],points_est[i],color_est_points,100);
            pcdpoints_est.insert(pcdpoints_est.end(),pp.begin(),pp.end());
        }

        if (cml["-out_pcd"]){
            vector<cv::Vec4f> pcdpoints=pcdpoints_gt;
            pcdpoints.insert(pcdpoints.end(),pcdpoints_est.begin(),pcdpoints_est.end());
            std::ofstream filePCD ( cml("-out_pcd"), std::ios::binary );
            filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<pcdpoints.size()<<"\nHEIGHT 1\nPOINTS "<<pcdpoints.size()<<"\nDATA binary\n";
            filePCD.write((char*)&pcdpoints[0],pcdpoints.size()*sizeof(pcdpoints[0]));
        }
        if (cml["-out_pcd_gt"]){
            vector<cv::Vec4f> pcdpoints=pcdpoints_gt;
             std::ofstream filePCD ( cml("-out_pcd_gt"), std::ios::binary );
            filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<pcdpoints.size()<<"\nHEIGHT 1\nPOINTS "<<pcdpoints.size()<<"\nDATA binary\n";
            filePCD.write((char*)&pcdpoints[0],pcdpoints.size()*sizeof(pcdpoints[0]));
        }
        if (cml["-out_pcd_est"]){
            vector<cv::Vec4f> pcdpoints=pcdpoints_est;
              std::ofstream filePCD ( cml("-out_pcd_est"), std::ios::binary );
            filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<pcdpoints.size()<<"\nHEIGHT 1\nPOINTS "<<pcdpoints.size()<<"\nDATA binary\n";
            filePCD.write((char*)&pcdpoints[0],pcdpoints.size()*sizeof(pcdpoints[0]));
        }


    }

//    if (cml["-gt_scaled_translated_log"]){
//        cout<<"-gt_scaled_translated_log"<<endl;
//        //transform the log locations  and save them
//        ofstream ofile_d(cml("-gt_scaled_translated_log"));
//        if (!ofile_d){cerr<<"could not open log output file"<<endl;exit(0);}
//        cv::Mat scaleMatrix=cv::Mat::eye(4,4,CV_32F);
//        scaleMatrix.at<float>(0,0)=scaleMatrix.at<float>(1,1)=scaleMatrix.at<float>(2,2)=scale_best;
//        for(auto &fp:f_gt){
//            fp.second=best_T* scaleMatrix*fp.second;
//            double tx,ty,tz,qx,qy,qz,qw;
//            getQuaternionAndTranslationfromMatrix44(fp.second,qx,qy,qz,qw,tx,ty,tz);
//            ofile_d<<fp.first<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;
//            cerr<<"d:"<<fp.first<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;

//        }
//        //now, save to file


//    }

    if (cml["-mm"]){
        string cm_str=cml("-mm");
        for(auto &c:cm_str) if (c==':') c=' ';
        string in,out;stringstream sstr;sstr<<cm_str;sstr>>in>>out;
        cerr<<"reading:"<<in<<endl;

        cv::Mat scaleMatrix=cv::Mat::eye(4,4,CV_32F);
        scaleMatrix.at<float>(0,0)=scaleMatrix.at<float>(1,1)=scaleMatrix.at<float>(2,2)=scale_best;
        auto mm=aruco_mm::MarkerMapper::readFromFile(in);

        //transform all markers
        auto matrix=best_T* scaleMatrix;
        for(auto &maker:mm->getMarkerSet()){
            maker.second.rt_g2m=  best_T*maker.second.rt_g2m;
        }
        mm->saveToPcd(out,false);
        cerr<<"saving to "<<out<<endl;

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
