#include "markermapper.h"
#include "utils/utils3d.h"
using namespace std;

int main(int argc,char **argv){

    try{
         if (argc!=3){cerr<<"in.amm out.yml "<<endl;return -1;}
        auto mm=aruco_mm::MarkerMapper::readFromFile(argv[1]);
        mm->getMarkerMap().saveToFile(argv[2]);

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}



