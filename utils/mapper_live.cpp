#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <memory>
#include <opencv2/imgproc/imgproc.hpp>
#include "markermapper.h"
#include "debug.h"
#include "sglviewer.h"


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
void putText(cv::Mat &im,string text,cv::Point p ){
    float size=float(im.cols)/float(1280);
    float fact=float(im.cols)/float(1280);
    if (fact<1) fact=1;

    cv::putText(im,text,p,cv::FONT_HERSHEY_SIMPLEX, size,cv::Scalar(0,0,0),3*fact);
    cv::putText(im,text,p,cv::FONT_HERSHEY_SIMPLEX, size,cv::Scalar(125,255,255),1*fact);

}

void printMessages(cv::Mat &im,vector<std::string> &messages){

    cv::Point pos(10,20);
    for(auto &s:messages){
        putText(im,s,pos);
        pos.y+=20;
    }
}

int main(int argc,char **argv){
    try{
        CmdLineParser cml(argc,argv);

        if (argc<4 || cml["-h"]){
            cerr<<"Usage: (live | intput-video.avi[:v2.avi[:v3.avi...]]) camera_parameters.yml marker_size(meters)   [-ref reference_marker_id]  [-out basename] [-d maker_dictionary]"<<endl;
        cerr<<"\tDictionaries: "; for(auto dict:aruco::Dictionary::getDicTypes())    cerr<<dict<<" ";cerr<<endl;
        return -1;
        }

        aruco_mm::debug::Debug::setLevel(5);

        string outBaseName= cml("-out","markerset");

        aruco::CameraParameters Camera;
        Camera.readFromXMLFile(argv[2]);
        float markerSize=atof(argv[3]);
        int ref_Marker_Id=-1;
         if (cml["-ref"]) ref_Marker_Id=stoi( cml("-ref"));
         int waitTime=5;
        //start processing
         aruco::MarkerDetector Mdetector;
         Mdetector.setDictionary(cml("-d","ALL_DICTS"));
        std::shared_ptr<aruco_mm::MarkerMapper> AMM;

        AMM=aruco_mm::MarkerMapper::create( );
        AMM->setParams(Camera,markerSize,ref_Marker_Id);
        //set the type of dictionary the markers belong to (by default it is ARUCO)

        cerr<<"Press esc to end video processing"<<endl;
        cerr<<"Press 's' to start/stop video"<<endl;
        cerr<<"Press 'a' to add a frame to the set"<<endl;
        cerr<<"Press 'k' skip 20 frames"<<endl;
        cerr<<"Press 'l' skip 100 frames"<<endl;

         char key=0;
        int frame=0;


        vector<string> messagesToPrintInImage(1);
        messagesToPrintInImage[0]="press 'a' to add images";
             cv::Mat image,img_resized;
            cv::VideoCapture vcap;
            if (string(argv[1]).find( "live")!=std::string::npos) {
                auto TheInputVideo=string(argv[1]);
                int vIdx = 0;
                // check if the :idx is here
                char cad[100];
                if (TheInputVideo.find(":") != string::npos)
                {
                    std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
                    sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
                }
                cout << "Opening camera index " << vIdx << endl;
                vcap.open(vIdx);
             }
            else vcap.open(argv[1]);
            if (!vcap.isOpened()){cerr<<"Could not open input"<<endl;return -1;}
            //wait until valid images came
            while( image.empty()) {vcap.grab();vcap.retrieve(image);}

            int frameCounter=0;
             do{
                vcap.retrieve(image);
                auto markers=Mdetector.detect(image);
                 for(auto &m:markers) m.draw(image);
                cv::resize(image,img_resized,cv::Size(min( image.cols,1080),min(image.rows,720)));
                printMessages(img_resized,messagesToPrintInImage);
                cv::imshow("image",img_resized);
                key=cv::waitKey(waitTime);
                if (key=='s') waitTime=waitTime==0? 5:0;
                if (key=='a') {
                    string name=std::to_string(frameCounter++);
                    while(name.size()<5) name="0"+name;
                    name="image-"+name+".jpg";
                    cv::imwrite(name,image);
                    aruco_mm::debug::Debug::clearStringDebugInfo();
                    bool res=AMM-> process ( markers ,-1,true);
                    if (res){
                        messagesToPrintInImage.back()="Frame added";
                     }
                    else
                        messagesToPrintInImage.back()="Error:"+aruco_mm::debug::Debug::getStringDebugInfo();
                }
                //skip
                if (key=='k')  for(int i=0;i<20;i++) {vcap.grab();frame++;}
                if (key=='l')  for(int i=0;i<100;i++) {vcap.grab();frame++;}
                //finish
                if(key==27) break;


            }while(vcap.grab() && key!=27);
            cout<<"Finish processing video."<<endl;

        cout<<"Starting optimization."<<endl;


        if (AMM->getFrameSet().size()==0)return EXIT_FAILURE;
        AMM->optimize();
        AMM->saveToPcd(outBaseName+".pcd",true);
        AMM->saveFrameSetPosesToFile(outBaseName+".log");
        AMM->getMarkerMap().saveToFile(outBaseName+".yml");
        AMM->getCameraParams().saveToFile(outBaseName+"-cam.yml");



        cout<<"Data is saved to "<<outBaseName<<"(.amm,pcd,log,yml)"<<endl;
        cout<<"The marker map generated is "<<outBaseName<<".yml and can be used with ArUco"<<endl<<endl;
        cout<<"You can see a 3D view of the map using the mapper_viewer program passing "<<outBaseName<<".yml as argument when you want"<<endl;

        OpenCvMapperViewer Viewer;
        aruco::MarkerMap mmap;
        mmap.readFromFile(outBaseName+".yml");
        Viewer.setParams(mmap,0.5,1280,960,"map_viewer");
        key=0;
        while(key!=27)
            key =     Viewer.show( );

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }



}
