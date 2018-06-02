#include <aruco/markermap.h>
#include "sglviewer.h"
#include <iostream>
using namespace std;
int main(int argc,char **argv){

    try{
        if (argc!=2){
            cerr<<"Usage: markermap.yml"<<endl;
            return -1;
        }
        aruco::MarkerMap mmap;
        mmap.readFromFile(argv[1]);
        OpenCvMapperViewer Viewer;
        Viewer.setParams(mmap,1.5,1280,960,"map_viewer");
        int key=0;
        while(key!=27)
            key =     Viewer.show( );

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
