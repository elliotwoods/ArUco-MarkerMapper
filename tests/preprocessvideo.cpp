#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/cameraparameters.h>
#include <aruco/dictionary.h>
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

        if (argc<5 || cml["-h"]){
            cerr<<"Usage: (live | intput-video.avi[:v2.avi[:v3.avi...]]) camera_parameters.yml marker_size(meters) out.ppv  [-fi frameIncrement] [-c :use for markers with corners] [-d dictionary]"<<endl;
            cerr<<"\tDictionaries: "; for(auto dict:aruco::Dictionary::getDicTypes())    cerr<<dict<<" ";cerr<<endl;
            return -1;}
        string outBaseName= argv[4];

        aruco::CameraParameters Camera;
        Camera.readFromXMLFile(argv[2]);
        float markerSize=atof(argv[3]);
        int frameIncrement=stoi( cml("-fi","1"));
        int waitTime=10;
        //start processing
        aruco_mm::debug::PreprocessedVideo video_processing;

        video_processing.setParams(Camera,markerSize);
        aruco::MarkerDetector::Params params;
        if( cml["-d"])
            video_processing._mdetector.setDictionary(cml("-d"));

        params.minSize=0.020; //params._minSize_pix=std::numeric_limits<int>::max();
//        params.useLowresImageForRectDetection=false;
//                if (cml["-c"]){//for boards/ or markers with corners
//                    params.borderDistThres=.01;//acept markers near the borders
////                    params._thresParam1=5;
////                    params._thresParam1_range=20;//search in wide range of values for param1
////                    params._cornerMethod=aruco::MarkerDetector::SUBPIX;//use subpixel corner refinement
////                    params._subpix_wsize= 5 ;//search corner subpix in a 5x5 widow area
//                }
        //video_processing._mdetector.setParams(params);//set the params above
        cerr<<"Press esc to end processing"<<endl;
         char key=0;
        uint32_t frame=0;

        auto videos=parseVideoFiles(argv[1]);

        for(auto vfile:videos){
            cv::Mat image,image2,img_resized;
            cv::VideoCapture vcap;
            if (string(argv[1])=="live") vcap.open(0);
            else vcap.open(vfile);
            if (!vcap.isOpened()){cerr<<"Could not open input"<<endl;return -1;}
            //wait until valid images came
            while( image.empty()) {vcap.grab();vcap.retrieve(image);}



            //        while(frame<6924){vcap.grab();frame++;}

            do{
                vcap.retrieve(image);
                image.copyTo(image2);

                frame++;
                if( frame%frameIncrement!=0) continue;

                video_processing. process ( image ,frame);
                video_processing. drawDetectedMarkers ( image2);
                cv::resize(image2,img_resized,cv::Size(min( image2.cols,1080),min(image2.rows,720)));
                cv::imshow("image",image2);
                key=cv::waitKey(waitTime);
                if (key=='s') waitTime=waitTime==0? 5:0;
                if (key=='w') cv::imwrite("image.jpg",image2);
                if (key=='k')  for(int i=0;i<20;i++) {vcap.grab();frame++;}
                if (key=='l')  for(int i=0;i<100;i++) {vcap.grab();frame++;}
                if(key==27) break;

                cout<<"frame "<<frame<<endl;
            }while(vcap.grab() && key!=27);
            cout<<"Finish processing video."<<endl;
        }
        cout<<"saving to"<<outBaseName<<endl;

        video_processing.saveToFile(outBaseName);


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }



}
