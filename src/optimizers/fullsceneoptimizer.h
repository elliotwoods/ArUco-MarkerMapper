#ifndef FULLSCENE_OPTIMZIZER_ARUCO_MM
#define FULLSCENE_OPTIMZIZER_ARUCO_MM
#include <opencv2/core/core.hpp>
#include <fstream>
#include "../marker_mapper_exports.h"
#include "sparselevmarq.h"
#include "../mapper_types.h"
namespace aruco_mm{
class MARKERMAPPER_API FullSceneOptimizer{
public:

    struct Params{
        int max_iters=200;
        double min_error=0.1;
        std::set<uint32_t> fixedMarkers;
        std::set<uint32_t> fixedFrames;
        bool fix_camera_params=false;
        bool verbose=false;
        double min_step_error=1;
        std::function<float(float)> error_func= [](float x){return x;};

    };
    double optimize(MarkerSet & MarkerSet, FrameSet & FrameSet,  const std::set<uint32_t> &fixedMarkers, aruco::CameraParameters &camp, bool fix_cameraparams);
    double optimize(MarkerSet & MarkerSet, FrameSet & FrameSet,  aruco::CameraParameters &camp,Params &params);
    void  error_optimization(const SparseLevMarq<double>::eVector &sol,SparseLevMarq<double>::eVector &error);
    void error_jacobian(const SparseLevMarq<double>::eVector  & sol,  Eigen::SparseMatrix<double> &sp);
    SparseLevMarq<double>::eVector  toVector(MarkerSet & MarkerSet, FrameSet & FrameSet,  aruco::CameraParameters &params);
    void fromVector(SparseLevMarq<double>::eVector  &sol,MarkerSet & MarkerSet, FrameSet & FrameSet,   aruco::CameraParameters &cam_params);
    std::vector<Eigen::Triplet<double> > _jacobian;
    MarkerSet sba_markerSet;
    FrameSet sba_frameSet;
    Params sba_ba_params;
    cv::Mat getRTMatrix ( SparseLevMarq<double>::eVector  & rt );
    std::vector<cv::Point3f> getMarkerPoints(float size,cv::Mat RT );
    std::vector<float> getSolution(MarkerSet & MarkerSet, FrameSet & FrameSet,  aruco::CameraParameters &cam_params);
private:
     void stepcallbackfunc(const SparseLevMarq<double>::eVector  &sol) ;
    void  addRT(  SparseLevMarq<double>::eVector &sol,int start,const cv::Mat &r,const cv::Mat &t);
    void  getRT(const SparseLevMarq<double>::eVector &sol,int start,cv::Mat &r,cv::Mat &t);
    void  stats( SparseLevMarq<double>::eVector  &error);
    template<typename T>
    T   get( std::map<uint32_t,T> &fs,int idx ){
        auto it=fs.begin();
        for(int i=0;i<idx;i++)it++;
          return it->second;
    }

    std::ofstream outErrorFile;//use only for debuggin;
    int it_debug;
 };
}


#endif
