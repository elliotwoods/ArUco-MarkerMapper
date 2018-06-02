#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <depthmaps/depthmap.h>
#include <depthmaps/pointcloud.h>
using namespace std;

float tofcolor(uchar r,uchar g,uchar b){
    float f=0;
    char *c=(char*)&f;
    c[0]=r;
    c[1]=g;
    c[2]=b;
    return f;
}

int main(int argc,char **argv){

    if (argc!=3){cerr<<"USage in.pcd out.pcd"<<endl;return -1;}
    depthmaps::DepthMap dm;
    depthmaps::PointCloud points;
    dm.readFromFile(argv[1]);
    int idx=0;
    for(int i=0;i<dm.rows;i++){
        auto ptr=dm.ptr<depthmaps::DepthPixel>(i);
        for(int j=0;j<dm.cols;j++)
              if(idx++%10==0) points.push_back(ptr[j]);
    }

    ofstream filePCD(argv[2]);
       filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points.size()<<"\nHEIGHT 1\nPOINTS "<<points.size()<<"\nDATA binary\n";

       filePCD.write((char*)&points[0],points.size()*sizeof(points[0]));


}
