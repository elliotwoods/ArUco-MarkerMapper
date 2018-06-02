#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/cameraparameters.h>
#include <aruco/dictionary.h>
#include <memory>
#include <opencv2/imgproc/imgproc.hpp>
#include "markermapper.h"
#include "debug.h"
#include "sglviewer.h"
#include "aruco/markerlabeler.h"

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

        if (argc<4 || cml["-h"]){
            cerr<<"Usage: (live | intput-video.avi[:v2.avi[:v3.avi...]]) camera_parameters.yml marker_size(meters)   [-ref reference_marker_id] [-fi frameIncrement] [-out basename] [-d maker_dictionary] [-c arucoConfig.yml]"<<endl;
        cerr<<"\tDictionaries: "; for(auto dict:aruco::Dictionary::getDicTypes())    cerr<<dict<<" ";cerr<<endl;
        return -1;
        }

     //   aruco_mm::debug::Debug::setLevel(5);

        string outBaseName= cml("-out","markerset");

        aruco::CameraParameters Camera;
        Camera.readFromXMLFile(argv[2]);
        float markerSize=atof(argv[3]);
        int ref_Marker_Id=-1;
        int frameIncrement=1;
        if (cml["-ref"]) ref_Marker_Id=stoi( cml("-ref"));
        if (cml["-fi"]) frameIncrement=stoi( cml("-fi"));
        int waitTime=5;
        //start processing
        std::shared_ptr<aruco_mm::MarkerMapper> AMM;
        AMM=aruco_mm::MarkerMapper::create( );
        AMM->setParams(Camera,markerSize,ref_Marker_Id);
        if (cml["-c"])
            AMM->getMarkerDetector().loadParamsFromFile(cml("-c"));
        else
         AMM->getMarkerDetector().setDictionary(cml("-d","ALL_DICTS"));

        cout<<AMM->getMarkerDetector().getMarkerLabeler()->getName()<<" "<<cml("-d","ALL_DICTS")<<endl;

        cerr<<"Press esc to end video processing"<<endl;
        cerr<<"Press 's' to start/stop video"<<endl;
        cerr<<"Press 'w' to take a snapshot"<<endl;
        cerr<<"Press 'k' skip 20 frames"<<endl;
        cerr<<"Press 'l' skip 100 frames"<<endl;

         char key=0;
        int frame=0;

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
                AMM-> process ( image ,frame);
                AMM-> drawDetectedMarkers ( image2);
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
        cout<<"Starting optimization."<<endl;


        AMM->optimize();


        //        AMM->saveToFile(outBaseName+".amm");
        AMM->saveToPcd(outBaseName+".pcd",true);
        AMM->saveFrameSetPosesToFile(outBaseName+".log");
        AMM->getMarkerMap().saveToFile(outBaseName+".yml");
        AMM->getCameraParams().saveToFile(outBaseName+"-cam.yml");

        if (!cml["-noshow"]){

            cout<<"Data is saved to "<<outBaseName<<"(.amm,pcd,log,yml)"<<endl;
            cout<<"The marker map generated is "<<outBaseName<<".yml and can be used with ArUco"<<endl<<endl;
            cout<<"You can see a 3D view of the map using the mapper_viewer program passing "<<outBaseName<<".yml as argument when you want"<<endl;

            OpenCvMapperViewer Viewer;
            aruco::MarkerMap mmap;
            mmap.readFromFile(outBaseName+".yml");
            Viewer.setParams(mmap,1.5,1280,960,"map_viewer");
            key=0;
            while(key!=27)
                key =     Viewer.show( );
        }

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }



}
