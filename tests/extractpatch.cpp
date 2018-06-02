#include <iostream>
#include <depthmaps/depthmap.h>
#include <depthmaps/depthmaputils.h>
#include <depthmaps/pointcloud.h>
#include "markermapper.h"
using namespace std;
cv::Point3f getCenter(const vector<cv::Point3f> &v){
    cv::Point3f center(0,0,0);
    for(auto p:v)center+=p;
    return center*(1./double(v.size()));
}
int main(int argc,char **argv){

    if(argc!=4){cerr<<"in.pcd in.amm out.pcd"<<endl;return -1;}
    depthmaps::DepthMap in,out;
    in.readFromFile(argv[1]);
    auto mm=aruco_mm::MarkerMapper::readFromFile(argv[2]);
    depthmaps::PointCloud pc;
    for(auto m:mm->getMarkerSet()){
        auto center=getCenter(m.second.get3dPoints());
        for(int y=0;y<in.rows;y++){
            depthmaps::DepthPixel *dp=in.ptr<depthmaps::DepthPixel>(y);
            for(int x=0;x<in.cols;x++){
                if (cv::norm( dp[x].toPoint3f()-center)   <0.17)pc.push_back(dp[x]);// cout<<dp[x].toPoint3f()<<endl;
            }
        }

    }
    pc.saveToPCD(argv[3]);
}
