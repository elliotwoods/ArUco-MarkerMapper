#include "markermapper.h"
#include "utils/utils3d.h"
using namespace std;

class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                                    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };

int main(int argc,char **argv){

    try{
         if (argc!=3){cerr<<"in.amm out.pcd "<<endl;return -1;}
        auto mm=aruco_mm::MarkerMapper::readFromFile(argv[1]);
        cout<<"#nmarekrs="<<mm->getMarkerSet().size()<<" nframes="<<mm->getFrameSet().size()<<endl;

        mm->saveToPcd(argv[2]);

//        for(auto frame:mm->getFrameSet()){
//            aruco_mm::MarkerMapper mp;
//            mp.getMarkerSet()=mm->getMarkerSet();

//             mp.getFrameSet().insert(frame);
//            mp.saveToPcd("out.pcd",true);;
//            cout<<"frame:"<<frame.first<<endl;
//            system("pcl_viewer out.pcd -multiview 1 -cam 14.646,20.5026/-0.394374,-0.35771,1.40675/1.91611,16.68,4.38336/-0.00669104,0.172977,-0.984903/0.8575/960,998/318,1177 &");
//            cin.ignore();
//        }

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}


