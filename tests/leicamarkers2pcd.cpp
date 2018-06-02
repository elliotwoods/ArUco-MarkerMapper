
#include <iostream>
#include <iomanip>
#include <fstream>
#include "utils/se3.h"
#include "markermapper.h"
#include "utils/utils3d.h"
#include "optimizers/sparselevmarq.h"
using namespace std;

std::map<uint32_t,vector<cv::Point3f>> getRealLeicaMarkers(string fp)throw(std::exception){
    std::map<uint32_t,vector<cv::Point3f> > mset;
    ifstream file(fp);
    if (!file) throw std::runtime_error("NO intpu feile");

    while(!file.eof())    {
        int id;
        if(! (file>>id)) break;
        vector<cv::Point3f> points(4);
        cv::Point3f center(0,0,0);
        for(int i=0;i<4;i++){
            file>>points[i].x>>points[i].y>>points[i].z;
        }
        mset[id]=points;
    }
    return mset;
}


int main(int argc,char **argv){
    try{
    auto allPoints=getRealLeicaMarkers(argv[0]);


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
