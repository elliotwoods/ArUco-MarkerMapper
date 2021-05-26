#ifndef __ARUCMM_Debug_H
#define __ARUCMM_Debug_H
#include <iostream>
#include <fstream>
#include <ctime>
#include "mapper_types.h"
#include <aruco/markerdetector.h>
#include <aruco/cameraparameters.h>
#include "marker_mapper_exports.h"
#include <mutex>
namespace aruco_mm{

namespace debug{
class MARKERMAPPER_API  Debug{
private:
    static  int level;//0(no debug), 5 medium, 10 high,>10 specific debug functions
    static  bool isInited;
    static std::mutex _debug_mutex;
    static std::string _debug_string;
 public:
    static void init();
    static void setLevel(int l);
    static int getLevel();


    static std::string getStringDebugInfo(bool emptyStringAfterCall=true);
    static void appendStringDebugInfo(const std::string &str);
    static void clearStringDebugInfo( );

    static std::string getFileName(std::string filepath){
        //go backwards until finding a separator or start
        int i;
        for( i=filepath.size()-1;i>=0;i--){
            if ( filepath[i]=='\\' || filepath[i]=='/') break;
        }
        std::string fn;fn.reserve( filepath.size()-i);
        for(size_t s=i;s<filepath.size();s++)fn.push_back(filepath[s]);
        return fn;
    }

#ifdef WIN32
  #define __func__ __FUNCTION__
#endif

};


#define markermapper_debug_msg(x,level) {std::stringstream sstr; sstr<<x<<std::endl; \
   aruco_mm::debug::Debug::init();\
   if (aruco_mm::debug::Debug::getLevel()>=level){\
         aruco_mm::debug::Debug::appendStringDebugInfo(sstr.str());\
         std::cout<<__func__<<":"<< aruco_mm::debug::Debug::getFileName(__FILE__)<<":"<<__LINE__  <<":  "<<x<<std::endl; \
        }}





//For internal usage only
class PreprocessedVideo{
public:
    void setParams(const aruco::CameraParameters &cp,double msize){_cam_params=cp;_markerSize=msize;}
    void process(const cv::Mat &img,uint32_t frame_idx);

    void saveToFile(std::string path);
    void readFromFile(std::string path);
    FrameSet _frameSet;
    aruco::CameraParameters _cam_params;
    aruco::MarkerDetector _mdetector;
    double _markerSize;


    void drawDetectedMarkers ( cv::Mat &imshow,int border_width=1) ;

private:
    arucoMarkerSet detected_markers;
};

}

}

#endif

