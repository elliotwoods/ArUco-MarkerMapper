#include "markermapper.h"
#include "mappers/globalgraph_markermapper.h"
#include "optimizers/fullsceneoptimizer.h"
using namespace std;

class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                                    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };

int main(int argc,char **argv){

    try{
        CmdLineParser cml(argc,argv);
        if (argc<4){cerr<<"in.amm out.amm (0:full 1:lagragian) [-it <val>:nofiters] [-v verbose] [-fi : fix intrinsics] [-mst <val>: min step error]"<<endl;return -1;}
        auto mm=aruco_mm::MarkerMapper::readFromFile(argv[1]);

        aruco_mm::GlobalGraphMarkerMapper *gmm= dynamic_cast<aruco_mm::GlobalGraphMarkerMapper*>(mm.get());
//        gmm->getOptimizingFrameSet(gmm->getFrameSet(),gmm->getMarkerSet(),gmm->getCameraParams(),1e-6,2);
//        exit(0);

            aruco_mm::FullSceneOptimizer mopt;
            aruco_mm::FullSceneOptimizer::Params params;
            params.max_iters=stoi(cml("-it","100"));
            params.min_step_error=stod(cml("-mste","0"));
            params.verbose=cml["-v"];
            params.fix_camera_params=cml["-fi"];
            params.fixedMarkers={mm->getOriginMarkerId()};
            mopt.optimize(mm->getMarkerSet(),mm->getFrameSet(),mm->getCameraParams(),params);


        mm->saveToFile(argv[2]);
        mm->saveToPcd(argv[2]+string(".pcd"));


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}

