#include "markermapper.h"
#include "debug.h"
#include "utils/utils3d.h"
#include <set>
#include <Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unordered_set>
#include <iomanip>
#include <fstream>
#include <random>
#include <list>
#include "optimizers/fullsceneoptimizer.h"
#include "mappers/globalgraph_markermapper.h"
#include "aruco/aruco.h"
#include "aruco/markerlabeler.h"
namespace aruco_mm{
using namespace std;

bool MarkerMapper::process(const cv::Mat &image, int frame_idx  , bool forceConnectedComponent) {
    markermapper_debug_msg ( "####start "<<frame_idx,5 );
    arucoMarkerSet markers;
    _mdetector.detect ( image,markers );


   // if (markers.size()<2)return;

    return process(markers,frame_idx,forceConnectedComponent);
}


void MarkerMapper:: optimize (bool blocking)  {
    if (_frameSet.size()==0)return;
    _finishedProcessing=false;
    if (blocking)
            optimize_wrap();
        else{
            _optimizingThread=std::thread([&](){this->optimize_wrap();});
    }
}

bool MarkerMapper::isOptimizationFinished(){
    return _finishedProcessing;
}

void MarkerMapper::waitForOptimizationFinished(){
    _optimizingThread.join();
}

void MarkerMapper::setParams (const aruco::CameraParameters & cam_p, float defaultMarkerSize, int originMarkerId, bool optimizeCameraIntrinsics)  {
    _cam_params=cam_p;
    _defaultMarkerSize=defaultMarkerSize;
//#if ARUCO_VERSION_MAJOR == 3

//#else
//    aruco::MarkerDetector::Params params;
//    params._thresParam1=15;
//    params._thresParam1_range=0;
//    params._thresParam2=10;
//    params._cornerMethod=aruco::MarkerDetector::LINES;
//    _mdetector.setParams(params);
//#endif
    _optimizeCameraIntrinsics=optimizeCameraIntrinsics;
    _originMarkerId=originMarkerId;
    _markerSet.clear();
}


arucoMarkerSet MarkerMapper::deleteAllBut( arucoMarkerSet &in,const std::set<uint32_t> &tokeep,const std::set<uint32_t> &tokeep2){

   arucoMarkerSet mm;
    for(auto m:in)
        if(tokeep.find(m.id)!=tokeep.end() || tokeep2.find(m.id)!=tokeep2.end()) mm.push_back(m);
    return mm;
}
void MarkerMapper::computeFrameSetPoses( ){
    computeFrameSetPoses(_frameSet,_markerSet,_cam_params);
}

void MarkerMapper::computeFrameSetPoses( FrameSet &fset_io, MarkerSet &mset,aruco::CameraParameters cam_params,bool onlyUnknown){
    //get the set of ids
    std::set<uint32_t> ids_set;
    for(auto m:mset) ids_set.insert(m.first);
    for(auto &frame:fset_io){
        if ( (onlyUnknown && frame.second.rt_c2g.empty()) || (!onlyUnknown) ){
         auto used_markers=MarkerMapper::deleteAllBut(frame.second.markers,ids_set);
         if (used_markers.size()>0){
             //get frame locations
             frame.second.rt_c2g= solvePnP(used_markers, mset,cam_params,true);
         }
         else frame.second.rt_c2g=cv::Mat();
        }
    }
}

double MarkerMapper::estimateCurrentLocation(std::vector<cv::Point3f> &objPoints,std::vector<cv::Point2f> &imgPoints,cv::Mat &R,cv::Mat &T,bool useRansac){

//    cout<<"npoints="<<objPoints<<endl;
//    cv::solvePnPRansac(objPoints ,imgPoints,_cam_params.CameraMatrix,_cam_params.Distorsion,R,T);
    if (useRansac)
        cv::solvePnPRansac(objPoints ,imgPoints,_cam_params.CameraMatrix,_cam_params.Distorsion,R,T);
 else   cv::solvePnP(objPoints ,imgPoints,_cam_params.CameraMatrix,_cam_params.Distorsion,R,T);
 //analyze the reprojection errors are eliminate erroneous locations
cout<<"sol xx  "<<R<<" "<<T<<endl;

//cout<<"Reprj error:";
vector<cv::Point2f> p2_reprj;
cv::projectPoints(objPoints,R,T,_cam_params.CameraMatrix,_cam_params.Distorsion,p2_reprj);
double avrg_err=0,minErr=std::numeric_limits<float>::max();
vector<double> error_marker(p2_reprj.size()/4,0);
for(size_t i=0;i<p2_reprj.size()/4;i++){
    for(int j=0;j<4;j++){
        error_marker[i]+=cv::norm(p2_reprj[i*4+j]-imgPoints[i*4+j]);
    }
    minErr=min(minErr,error_marker[i]);
    avrg_err+=error_marker[i];
}
cout<<endl;
avrg_err/=float(error_marker.size());
cout<<"ERR location="<<minErr<<endl;
return minErr;
}








std::vector<double> MarkerMapper::getRprjErr(const FrameInfo &fi,    MarkerSet &mset, const aruco::CameraParameters &cam)
{
    std::vector<double> errors;
    for(auto marker: fi.markers){
        if (mset.find(marker.id)!=mset.end()){
        vector<cv::Point2f> reprj_2d;
        cv::Mat r,t;
        aruco_mm::getRTfromMatrix44(fi.rt_c2g,r,t);
        cv::projectPoints(getMarkerPoints(mset[marker.id].markerSize,mset[marker.id].rt_g2m),r,t,cam.CameraMatrix,cam.Distorsion,reprj_2d);
        double sum=0;
        for(size_t i=0;i<reprj_2d.size();i++)
            sum+=cv::norm(reprj_2d[i]-marker[i]);
        sum/=double(reprj_2d.size());
        errors.push_back(sum);
    }
    }
    return errors;
}

double MarkerMapper::getRprjErr(const vector<cv::Point3f> &p3d,const  vector<cv::Point2f> &p2d ,aruco::CameraParameters &cam, const cv::Mat &rt){
    vector<cv::Point2f> reprj_2d;
    cv::Mat r,t;
    aruco_mm::getRTfromMatrix44(rt,r,t);
    cv::projectPoints(p3d,r,t,cam.CameraMatrix,cam.Distorsion,reprj_2d);
    double sum=0;
    for(size_t i=0;i<reprj_2d.size();i++)
        sum+=cv::norm(reprj_2d[i]-p2d[i]);
    sum/=double(reprj_2d.size());
    return sum;
}






double MarkerMapper::bundleAdjustment(){
    FullSceneOptimizer opt;
    return opt.optimize(_markerSet,_frameSet,{uint32_t(_originMarkerId)},_cam_params, false);
}





void  MarkerMapper::base_toStream ( ostream &str )  {

    str.write ( ( char* ) &_originMarkerId,sizeof ( _originMarkerId ) );
     str.write ( ( char* ) &_frameCounter,sizeof ( _frameCounter ) );
    str.write ( ( char* ) &_defaultMarkerSize,sizeof ( _defaultMarkerSize ) );
    IoHelper::toStream ( _cam_params.CameraMatrix,str );
    IoHelper::toStream ( _cam_params.Distorsion,str );
    str.write ( ( char* ) &_cam_params.CamSize,sizeof ( _cam_params.CamSize ) );
    IoHelper::toStream__(_frameSet,str);
    IoHelper::toStream__(_markerSet,str);


}

void MarkerMapper::print_debug_info(){
    cout<<" _originMarkerId="<<_originMarkerId<<endl;
    cout<<" _defaultMarkerSize="<<_defaultMarkerSize<<endl;
    cout<<"  _cam_params.CameraMatrix="<< _cam_params.CameraMatrix<<endl;
    cout<<" _cam_params.Distorsion="<<_cam_params.Distorsion<<endl;
    cout<<" _cam_params.CamSize="<<_cam_params.CamSize.width<<" "<<_cam_params.CamSize.height<<endl;
    cout<<"marker set"<<endl;
    for(auto m:_markerSet) cout<< m.second<<endl;
    cout<<"frame set"<<endl;
    for(auto f:_frameSet) cout<<f.second<<endl;

//    fromStream__<std::unordered_map<uint32_t,MarkerInfo>,uint32_t,MarkerInfo>(_markerSet,str);
}
ostream & operator<<(ostream &str,const FrameInfo &fi){
    str<<"frame_idx="<<fi.frame_idx<< " "<<se3(fi.rt_c2g)<<endl;
    for(auto m:fi.markers)str<<m<< " ";

    return str;
}
ostream & operator<<(ostream &str,const MarkerInfo &fi){
    str<<fi.id<<" "<<fi.rt_g2m<<" "<<fi.markerSize<<endl;
    return str;
}


void  MarkerMapper::base_fromStream ( istream &str )  {

    str.read( ( char* ) &_originMarkerId,sizeof ( _originMarkerId ) );
    str.read ( ( char* ) &_frameCounter,sizeof ( _frameCounter ) );
    str.read ( ( char* ) &_defaultMarkerSize,sizeof ( _defaultMarkerSize ) );
    IoHelper::fromStream ( _cam_params.CameraMatrix,str );
    IoHelper::fromStream ( _cam_params.Distorsion,str );
    str.read ( ( char* ) &_cam_params.CamSize,sizeof ( _cam_params.CamSize ) );
    IoHelper::fromStream__<FrameSet,uint32_t,FrameInfo>(_frameSet,str);
    IoHelper::fromStream__<MarkerSet,uint32_t,MarkerInfo>(_markerSet,str);
}
uint64_t easyhash(const std::string  &str){
uint64_t sum=0;
int i=1;
for(auto c:str) sum+=c*(i++);
return sum;
}
void MarkerMapper::saveToFile ( std::string fname ) {
    ofstream file ( fname.c_str(),ios::binary );
    if ( !file )  throw std::runtime_error ( "Could not open file:"+fname );


    //magiv
    uint64_t magic=192831918;
    file.write ( ( char* ) &magic,sizeof ( magic ) );
    //now, write a signature
    uint64_t sig = easyhash( getName());
    file.write ( ( char* ) &sig,sizeof ( sig ) );

    base_toStream ( file );
    toStream(file);

}


std::shared_ptr<MarkerMapper>  MarkerMapper::readFromFile ( std::string fname ) {
    ifstream file ( fname.c_str(),ios::binary );
    if ( !file )  throw std::runtime_error ( "Could not open file:"+fname );

    //magiv
    uint64_t magic;
    file.read ( ( char* ) &magic,sizeof ( magic ) );
    if ( magic!=192831918 )    throw std::runtime_error ( "Invalid file type :"+fname );
    uint64_t sig;
    file.read ( ( char* ) &sig,sizeof ( sig ) );
    //find the class with this signature
    auto names=getAvailableMarkerMappers();
    std::shared_ptr<MarkerMapper> ptr;
    for(auto name:names) if (easyhash( name)==sig) ptr= create(name);
    if (!ptr) throw std::runtime_error("Could not read any calid mapper from file:"+fname);

    //read common info
    ptr->base_fromStream ( file );
    ptr->fromStream(file);
    return ptr;

}


 void MarkerMapper::saveToPcd(string fpath,bool addViewLocations)throw(std::exception) {
     if (addViewLocations)
         savePCDFile(fpath,_markerSet,_frameSet,_originMarkerId);
     else
         savePCDFile(fpath,_markerSet,_originMarkerId);
 }
 void MarkerMapper::getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,float &qx,float &qy,float &qz,float &qw,float &tx,float &ty,float &tz){
     //get the 3d part of matrix and get quaternion
     assert(M_in.total()==16);
     cv::Mat M;M_in.convertTo(M,CV_32F);
     cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
     //use now eigen
     Eigen::Matrix3f e_r33;
     for(int i=0;i<3;i++)
         for(int j=0;j<3;j++)
             e_r33(i,j)=M.at<float>(i,j);

     //now, move to a angle axis
     Eigen::Quaternionf q(e_r33);
     qx=q.x();
     qy=q.y();
     qz=q.z();
     qw=q.w();


     tx=M.at<float>(0,3);
     ty=M.at<float>(1,3);
     tz=M.at<float>(2,3);


 }
 void MarkerMapper::saveFrameSetPosesToFile(string filepath )throw(std::exception){
     std::ofstream file(filepath);
     float qx,qy,qz,qw,tx,ty,tz;
     for(auto frame:_frameSet){
         if ( !frame.second.rt_c2g.empty()){
             auto m=frame.second.rt_c2g.inv();
             getQuaternionAndTranslationfromMatrix44(m,qx,qy,qz,qw,tx,ty,tz);
             file<<frame.first<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;
          //   cout<<m<<" "<<tx<<" "<<ty<<" "<<tz<<endl;
             //cin.ignore();
         }
     }
 }






 double bundleAdjustment(MarkerSet & MarkerSet, FrameSet & FrameSet,  const std::set<uint32_t> &fixedMarkers,aruco::CameraParameters &camp, bool fix_cameraparams){
     FullSceneOptimizer opt;
     return opt.optimize(MarkerSet,FrameSet,fixedMarkers,camp,fix_cameraparams);
 }

 //returns the marker indicated. If not present, returns a marker with id std::numeric_limits<int>::max();
 MarkerInfo & MarkerMapper::operator[](int id){
     return _markerSet[id];
 }

 //returns the frame indicated. If not present, returns a marker with id std::numeric_limits<int>::max();
 FrameInfo & MarkerMapper::operator()(int id){
    return _frameSet[id];

 }



 //Returns the mapper with the name indicated
std::shared_ptr< MarkerMapper> MarkerMapper::create(string name )throw (std::exception){

if(name.empty()) name="global_graph";
if (name=="global_graph") return std::make_shared<GlobalGraphMarkerMapper>();/*
if (name=="global_tree") return std::make_shared<GlobalTreeMarkerMapper>();
else if (name=="global_graph") return std::make_shared<GlobalGraphMarkerMapper>();
else if (name=="incremental") return std::make_shared<IncrementalMarkerMapper>();
else if(name=="robust_incremental") return std::make_shared<RobustIncrementalMarkerMapper>();
else if(name=="incremental_graph")return std::make_shared<IncrementalGraphMarkerMapper>();*/
 else throw std::runtime_error("Invalid marker mapper:"+name);
}

aruco::MarkerMap MarkerMapper::getMarkerMap()  {
    aruco::MarkerMap mmap;
    mmap.setDictionary(_mdetector.getMarkerLabeler()->getName());
    mmap.mInfoType=aruco::MarkerMap::METERS;
    //go and transfer data to the map
    for(auto m:_markerSet){
        aruco::Marker3DInfo m3di;
        for(auto p:m.second.get3dPoints())
            m3di.push_back(p);
        m3di.id=m.first;
        mmap.push_back(m3di);
    }
    return mmap;
}


}




