#include "markermapper.h"
#include "debug.h"
#include "utils/utils3d.h"
 #include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;

class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                                    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };

int main(int argc,char **argv){
//    try{
//        if (argc!=4){cerr<<"usage: in.amm video out.amm"<<endl;return -1;}
//        CmdLineParser cml(argc,argv);
//        aruco_mm::debug::Debug::setLevel(10);
//        auto mm=aruco_mm::MarkerMapper::readFromFile(argv[1]);
//        cv::VideoCapture vcap(argv[2]);
//        if (!vcap.isOpened()){cerr<<"no video"<<endl;return -1;}
//        if (mm->getName()!="global_tree")
//            throw std::runtime_error("Input is not of type GlobalMarkerMapper");
//        auto gmm=dynamic_cast<aruco_mm::GlobalTreeMarkerMapper*>(mm.get());

//        aruco::MarkerDetector::Params params;
//        params._minSize=0.025; params._minSize_pix=std::numeric_limits<int>::max();
//        gmm->getMarkerDetector().setParams(params);


//        gmm->loadKdtreeFromFrameSets();
//        int fdix=gmm->getFrameSet().rbegin()->first;
//        cv::Mat image,image2,img_resized;
//        char key=0;
//        while(vcap.grab() && key!=27){
//            vcap.retrieve(image);
//            gmm->process(image,fdix++);
//            image.copyTo(image2);
//            gmm-> drawDetectedMarkers ( image2);
//            cv::resize(image2,img_resized,cv::Size(min( image2.cols,1080),min(image2.rows,720)));
//            cv::imshow("image",image2);
//            key=cv::waitKey(10);

//        }
//        gmm->saveToFile(argv[3]);

//    }catch(std::exception &ex){
//        cerr<<ex.what()<<endl;
//    }
}

