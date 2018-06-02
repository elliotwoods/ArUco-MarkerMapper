#include "mappers/posegraphoptimizer.h"
#include <iostream>
using namespace std;
int main(int argc,char **argv){

    aruco_mm::StgMatrix<aruco_mm::Pose_Error> pose_graph_io;
    ifstream file(argv[1],ios::binary);
    if (!file)throw std::runtime_error("COuld not open file");
    pose_graph_io.fromStream(file);
    aruco_mm::PoseGraphOptimizer pgo;
    pgo.optimize(pose_graph_io);
}
