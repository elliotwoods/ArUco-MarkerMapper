#include <iostream>
#include <iomanip>
#include <fstream>
#include "utils/se3.h"
#include "markermapper.h"
#include "utils/utils3d.h"
#include "optimizers/sparselevmarq.h"
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

vector<float> computeBestTransform(const std::map<uint32_t,cv::Point3f> &real,const aruco::MarkerMap &detected){

    //first, compute  horn to obtain initial estimation

    //get center of both
    vector<cv::Point3f> p3dreal,p3ddect;
    for(auto m:detected){
        if (real.count(m.id)){
            p3ddect.push_back(getCenter(m.points));
            p3dreal.push_back(real.at (m.id));
        }
    }

    double err;
    auto RT=    aruco_mm::rigidBodyTransformation_Horn1987(p3ddect,p3dreal,&err);
    cout<<"Initial err="<<err<<endl;
    double avrg=0;
    {
        for(size_t i=0;i<p3dreal.size();i++){
            auto p=aruco_mm::mult(RT,p3ddect[i]);
            avrg+=cv::norm(p-p3dreal[i]);
            cout<<cv::norm(p-p3dreal[i])<<" ";
        }

        cout<<" avrg="<<avrg/double(p3dreal.size())<<endl;
    }


    auto errFunct=[&](const aruco_mm::SparseLevMarq<float>::eVector &sol,aruco_mm::SparseLevMarq<float>::eVector &err){
        //Create the transform
        aruco_mm::se3 rt(sol(0),sol(1),sol(2),sol(3),sol(4),sol(5));
        float scale=sol(6);
        //move the points first and then scale
        cv::Mat m=rt;
         err.resize(p3ddect.size());
        //compute err
        for(size_t i=0;i<p3dreal.size();i++){
            auto p=scale*aruco_mm::mult(m,p3ddect[i]);
            err(i)= cv::norm(p - p3dreal[i]);
        }



    };

    aruco_mm::se3 initSol(RT);
    aruco_mm::SparseLevMarq<float>::eVector sol(7);
    for(int i=0;i<6;i++) sol(i)=initSol[i];
    sol(6)=1;
    aruco_mm::SparseLevMarq<float> solver;
    solver.setParams(100,1e-5,1e-7);
    solver.verbose()=true;
    cout<<sol.transpose()<<endl;
    solver.solve(sol,  std::bind(errFunct,std::placeholders::_1,std::placeholders::_2));
    cout<<sol.transpose()<<endl;
    //    ,std::bind(&MarkerSetLagragianOptimizer::jacobian,this,std::placeholders::_1,std::placeholders::_2));
    //final error
    {

        aruco_mm::se3 rt(sol(0),sol(1),sol(2),sol(3),sol(4),sol(5));
        float scale=sol(6);
        //move the points first and then scale
        cv::Mat RT=rt;
        //compute err
        double avrg=0;
        for(size_t i=0;i<p3dreal.size();i++){
            auto p=scale*aruco_mm::mult(RT,p3ddect[i]);
            auto err1=(p.x-p3dreal[i].x)*(p.x-p3dreal[i].x);
            err1+=(p.y-p3dreal[i].y)*(p.y-p3dreal[i].y);
            err1+=(p.z-p3dreal[i].z)*(p.z-p3dreal[i].z);
            avrg+= err1;
//            avrg+=cv::norm(p-p3dreal[i]);
            cout<<cv::norm(p-p3dreal[i])<<" ";
        }

        cout<<" avrg="<<sqrt(avrg/double(p3dreal.size()))<<endl;
    }



}

int main(int argc,char **argv)
{
    if (argc!=3){cerr<<"in.txt markermap.yml"<<endl;return -1;}
    auto mc=getRealLeicaCenters(argv[1]);



    aruco::MarkerMap mmap;
    mmap.readFromFile(argv[2]);

    computeBestTransform(mc,mmap);

    return -1;

}
