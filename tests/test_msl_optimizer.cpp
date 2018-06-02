#include "markermapper.h"
#include "optimizers/markersetlagragianoptimizer.h"

int main(int argc,char **argv){

    try{
        if (argc!=3){cerr<<"in.amm out.amm"<<endl;return -1;}
        auto mm=aruco_mm::MarkerMapper::create();
        mm->readFromFile(argv[1]);
        aruco_mm::MarkerSetLagragianOptimizer mopt;
        aruco_mm::MarkerSetLagragianOptimizer::Params params;
        params.fixedMarkers={mm->getOriginMarkerId()};
        mopt.optimize(mm->getMarkerSet(),mm->getFrameSet(),mm->getCameraParams(),params);
        mm->saveToFile(argv[2]);
        mm->saveToPcd(argv[2]+string(".pcd"));


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
