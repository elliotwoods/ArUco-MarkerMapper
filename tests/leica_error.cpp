#include <iostream>
#include <iomanip>
#include <fstream>
#include "markermapper.h"
#include "utils/utils3d.h"
using namespace std;

std::map<uint32_t,cv::Point3f> getRealLeicaCenters(string fp)throw(std::exception){
std::map<uint32_t,cv::Point3f> mset;
    ifstream file(fp);
    if (!file) throw std::runtime_error("NO intpu feile");

    while(!file.eof())    {
            int id;
            if(! (file>>id)) break;
             vector<cv::Point3f> points(4);
            cv::Point3f center(0,0,0);
            for(int i=0;i<4;i++){
                    file>>points[i].x>>points[i].y>>points[i].z;
                    center+=points[i];
            }
             center*=0.25;
            mset[id]=center;
    }
return mset;
}


cv::Point3f getCenter(const vector<cv::Point3f> &v){
cv::Point3f center(0,0,0);
for(auto p:v)center+=p;
return center*(1./double(v.size()));
}

aruco_mm::MarkerInfo resize(aruco_mm::MarkerInfo &mi,double f){
    //get the 3d points
    auto points=mi.get3dPoints();

    for(auto &p:points) p*=f;
    aruco_mm::MarkerInfo mres;
    mres.id=mi.id;
    mres.markerSize=cv::norm(points[0]-points[1]);;
    mres.rt_g2m= aruco_mm::rigidBodyTransformation_Horn1987( mres.get3dPoints(false), points);
    return mres;
}

aruco_mm::MarkerSet resize(aruco_mm::MarkerSet &ms,double f){
    aruco_mm::MarkerSet mres;
    for(auto m:ms)
        mres[m.first]=resize(m.second,f);
    return mres;
}


cv::Mat transform( std::map<uint32_t,cv::Point3f> &mc , aruco_mm::MarkerSet &ms,double *err){

    vector<cv::Point3f> real,est;
    for(auto m:mc){
        real.push_back(m.second);
        est.push_back(getCenter( ms[m.first].get3dPoints()));
    }
    return aruco_mm::rigidBodyTransformation_Horn1987(est,real,err);
}

double search(std::map<uint32_t,cv::Point3f> &mc,std::shared_ptr<aruco_mm::MarkerMapper> mm ,double start,double end,double step ){
    cout<<"search ["<<start<<","<<end<<":"<<step<<"]"<<endl;
    double minErr=std::numeric_limits<double>::max(),bestf=0;
    for(double f=start;f<=end;f+=step){
        auto mset_resized=resize(mm->getMarkerSet(),f);
        double err;
        transform(mc,mset_resized,&err);
        if (err<minErr){
            minErr=err;
            bestf=f;
        }
    }
    cout<<std::setprecision(10)<< "best f="<<bestf<<" err="<<minErr<<endl;
    return bestf;
}
double grid_search( std::map<uint32_t,cv::Point3f> &mc,std::shared_ptr<aruco_mm::MarkerMapper> mm ){
    double center=1;
    double size=0.05;
    double step=0.01;
    for(int i=0;i<4;i++){

        center=search(mc,mm,center-size,center+size,step);
        size=2*step;
        step*=0.1;
    }
    return center;
}

int main(int argc,char **argv)
{
    if (argc!=3){cerr<<"in.txt in.amm"<<endl;return -1;}
    auto mc=getRealLeicaCenters(argv[1]);
    auto mm=aruco_mm::MarkerMapper::readFromFile(argv[2]);
    double f=grid_search(mc,mm);


    //resize markers first
    mm->getMarkerSet()=resize(mm->getMarkerSet(),f);

    double err;
    cv::Mat real2est=transform(mc,mm->getMarkerSet(),&err);
    cout<<"err="<<err<<endl;
    for(auto &m:mm->getMarkerSet())
        m.second.rt_g2m=real2est*   m.second.rt_g2m ;
    mm->saveToPcd("res2.pcd");
    mm->saveToFile("res2.amm");
    double derr=cv::norm( mc[0] - mc[712]) - cv::norm( getCenter( mm->getMarkerSet()[0].get3dPoints())-getCenter( mm->getMarkerSet()[712].get3dPoints()) );
    cerr<<"errd="<< derr/double(cv::norm( mc[0] - mc[712])) *100<<"%"<<endl;



}
