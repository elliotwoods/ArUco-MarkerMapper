#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/cameraparameters.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "markermapper.h"
#include "debug.h"
using namespace std;

class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                                    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };

std::vector<std::string>  get_image_list(int argc,char **argv){
    std::vector<std::string> imags;
    for(int i=4;i<argc;i++)
        imags.push_back(argv[i]);
    return imags;
}


int main(int argc,char **argv){
    try{
        CmdLineParser cml(argc,argv);

        if (argc<5 || cml["-h"]){cerr<<"Usage:  camera_parameters.yml marker_size(meters) out.ppv  image1 image2 ..."<<endl;return -1;}
        string outBaseName= argv[3];

        aruco::CameraParameters Camera;
        Camera.readFromXMLFile(argv[1]);
        float markerSize=atof(argv[2]);
        int frameIncrement=stoi( cml("-fi","1"));
        //start processing
        aruco_mm::debug::PreprocessedVideo video_processing;

        video_processing.setParams(Camera,markerSize);
        aruco::MarkerDetector::Params params;
        params.minSize=0.01;

        //params._minSize_pix=std::numeric_limits<int>::max();
        params.borderDistThres=.01;//acept markers near the borders

//        params._thresParam1=5;
//        params._thresParam1_range=20;//search in wide range of values for param1
         if (cml["-c"]){//for boards/ or markers with corners
//             params.useLowresImageForRectDetection=true;
//             params._cornerMethod=aruco::MarkerDetector::SUBPIX;//use subpixel corner refinement
//            params._subpix_wsize= 5 ;//search corner subpix in a 5x5 widow area
        }
        //video_processing._mdetector.setParams(params);//set the params above
        cerr<<"Press esc to end processing"<<endl;
        char key;
        uint32_t frame=0;

        auto images=get_image_list(argc,argv);

        for(auto img_file:images){
            cv::Mat image,image2,img_resized;
            image=cv::imread(img_file);
            image.copyTo(image2);

            frame++;
            if( frame%frameIncrement!=0) continue;

            video_processing. process ( image ,frame);
            video_processing. drawDetectedMarkers ( image2);
            cv::resize(image2,img_resized,cv::Size(min( image2.cols,1080),min(image2.rows,720)));
            cv::imshow("image",img_resized);
            key=cv::waitKey(0);
            cout<<"frame "<<frame<<endl;
        }
        cout<<"saving to: "<<outBaseName<<endl;
        video_processing.saveToFile(outBaseName);
    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
