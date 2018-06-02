#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/cameraparameters.h>
#include <memory>
#include <opencv2/imgproc/imgproc.hpp>
#include "markermapper.h"
#include "debug.h"
using namespace std;
class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                                    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };

std::vector<std::string>  parseVideoFiles(string video){
    for(auto &c:video) if (c==':')c=' ';
    //now, parse
    stringstream sstr;sstr<<video;
    std::vector<std::string> videos;
    while(!sstr.eof()){
        std::string v;
        if (sstr>>v)  if (!v.empty()) videos.push_back(v);
    }
    return videos;
}


int main(int argc,char **argv){
    try{
        CmdLineParser cml(argc,argv);
        if (argc<3 || cml["-h"]){cerr<<"Usage: in.ppv[:in2.ppv:...]  out.mmp [-debug level] [-nofinalize] [-h: this help] [-noaddframes]"<<endl;return -1;}

        auto ppv_videos=parseVideoFiles(argv[1]);
        auto mapper=aruco_mm::MarkerMapper::create();
        int refId=stoi(cml("-ref","-1"));
        aruco_mm::debug::Debug::setLevel(stoi(cml("-debug","0")));
        if (!mapper) throw std::runtime_error("could not create mapper");
        uint32_t frameOff=0;
        if (!cml["-noaddframes"])
            for(int i=0;i<ppv_videos.size();i+=10){
                aruco_mm::debug::PreprocessedVideo ppv;
                ppv.readFromFile(ppv_videos[i]);
                cout<<"NFRAMES="<<ppv._frameSet.size()<<endl;
                cout<<"markersize="<<ppv._markerSize<<endl;
                if (i==0) mapper->setParams(ppv._cam_params,ppv._markerSize,refId);
                for(auto frame:ppv._frameSet)
                    mapper->process(frame.second.markers,frame.first+frameOff);

                frameOff+=ppv._frameSet.rbegin()->first;
            }


         if (!cml["-nofinalize"]) {
            mapper->optimize(false);
            while(!mapper->isOptimizationFinished()){

                std::cerr<<aruco_mm::debug::Debug::getStringDebugInfo();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            mapper->waitForOptimizationFinished();
        }

        mapper->getMarkerMap().saveToFile(string(argv[2])+".yml");
        mapper->saveToFile(string(argv[2])+".amm");
        mapper->saveToPcd(argv[2]+string(".pcd"),true);
        mapper->saveFrameSetPosesToFile(argv[2]+string(".log"));
        mapper->getCameraParams().saveToFile("camera-"+string(argv[2])+string(".yml"));

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }

}
