#include "debug.h"
#include <fstream>
namespace aruco_mm{
namespace debug{
using namespace  std;
int Debug::level=2;
std::mutex Debug::_debug_mutex;
std::string Debug::_debug_string;

bool Debug::isInited=false;

void Debug::setLevel ( int l ) {
    level=l;
    isInited=false;
    init();
}
int Debug::getLevel() {
    init();
    return level;
}
void Debug::init() {
    if ( !isInited ) {
        isInited=true;
        if ( level>=1 ) {
        }
    }

}
std::string Debug::getStringDebugInfo(bool emptyStringAfterCall){
    std::string  aux;
    std::unique_lock<std::mutex> lock(_debug_mutex);
    aux=_debug_string;
    if (emptyStringAfterCall) _debug_string.clear();
    return aux;
}

void Debug::appendStringDebugInfo(const std::string &str){
    std::unique_lock<std::mutex> lock(_debug_mutex);
    _debug_string+=str;
}
void Debug::clearStringDebugInfo(){
    std::unique_lock<std::mutex> lock(_debug_mutex);
    _debug_string.clear();

}
void PreprocessedVideo::process(const cv::Mat &image, uint32_t frame_idx){
     detected_markers=_mdetector.detect ( image );
    if (detected_markers.size()<2)return;
    _frameSet.insert(std::make_pair(frame_idx,FrameInfo(frame_idx,detected_markers)));

}

void PreprocessedVideo::saveToFile(std::string path){
ofstream file(path);
if (!file) throw std::runtime_error("Could not open file:"+path);
uint32_t sig=1871;
file.write((char*)&sig,sizeof(sig));

file.write((char*)&_markerSize,sizeof(_markerSize));
IoHelper::toStream ( _cam_params.CameraMatrix,file );
IoHelper::toStream ( _cam_params.Distorsion,file );
file.write ( ( char* ) &_cam_params.CamSize,sizeof ( _cam_params.CamSize ) );
IoHelper::toStream__(_frameSet,file);

}

void PreprocessedVideo::readFromFile(std::string path){


    ifstream file(path);
    if (!file) throw std::runtime_error("Could not open file:"+path);
    uint32_t sig;
    file.read((char*)&sig,sizeof(sig));
    if (sig!=1871)throw std::runtime_error("File is not of approrpiate type :"+path);

    file.read((char*)&_markerSize,sizeof(_markerSize));
    IoHelper::fromStream ( _cam_params.CameraMatrix,file );
    IoHelper::fromStream ( _cam_params.Distorsion,file );
    file.read( ( char* ) &_cam_params.CamSize,sizeof ( _cam_params.CamSize ) );
    IoHelper::fromStream__<FrameSet,uint32_t,FrameInfo>(_frameSet,file);
}
void PreprocessedVideo::drawDetectedMarkers ( cv::Mat &imshow,int border_width) {
    for ( auto &m:detected_markers )

            m.draw ( imshow,cv::Scalar ( 0,0,255 ) ,border_width);
}

}
}

