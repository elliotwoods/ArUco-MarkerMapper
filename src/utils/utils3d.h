#ifndef MatUtils_HPP_
#define MatUtils_HPP_

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <aruco/cameraparameters.h>
#include "../mapper_types.h"
#include <exception>
namespace aruco_mm {
class se3;
cv::Mat solvePnP(const  std::vector<aruco::Marker> &markers,    MarkerSet & MarkerSet,const aruco::CameraParameters &camp,bool ransac=false,cv::Mat initial_rt=cv::Mat());
cv::Mat solvePnP(const std::vector<cv::Point3f> &p3d,const std::vector<cv::Point2f> &p2d,const aruco::CameraParameters &cp,bool ransac=false,cv::Mat initial_rt=cv::Mat());

std::vector<cv::Point3f> getMarkerPoints(float size,cv::Mat RT=cv::Mat());
std::vector<cv::Point3f> getMarkerPoints( const MarkerInfo &m);
double cameraMarkerDotProduct(cv::Mat c2m);
std::map<uint32_t,double> getAvrgMarkerRepjError(const MarkerSet & ms, const FrameSet &fs, const aruco::CameraParameters &_cam_params);

MARKERMAPPER_API void savePCDFile(std::string fpath, const MarkerSet &ms, int origin=-1) ;
MARKERMAPPER_API void savePCDFile(std::string fpath, const MarkerSet &ms, const FrameSet &fset, int origin=-1, bool writeFrameIds=false, cv::Scalar marker_color=cv::Scalar(255,0,0), cv::Scalar frame_color=cv::Scalar(0,255,0)) ;

MARKERMAPPER_API void savePCDFile(std::string fpath,const MarkerSet &ms, const std::vector<cv::Mat >  &vector_rt_c2g,int org_id =-1) ;

void savePCDFile(const char *filename,  const std::vector<cv::Point3f> &points,
                 const  cv::Scalar color ) ;
void savePCDFile(const char *filename,  const std::vector<cv::Point3f> &points,
                 const std::vector<cv::Scalar> &colors) ;
void savePCDFile(std::string fpath,const std::vector<cv::Point3f> &points);
/**
    * Multiplication of m by p. m is a 4x4 matrix
    */
template <typename T,typename Point3dType>
Point3dType mult ( const cv::Mat &m,  Point3dType  p ) {
    assert ( m.isContinuous() );
    assert ( ( m.type() ==CV_32F && sizeof ( T ) ==4 ) || ( m.type() ==CV_64F && sizeof ( T ) ==8 ) );

    const T *ptr=m.ptr<T> ( 0 );
    Point3dType res;
    res.x= ptr[0]*p.x +ptr[1]*p.y +ptr[2]*p.z+ptr[3];
    res.y= ptr[4]*p.x +ptr[5]*p.y +ptr[6]*p.z+ptr[7];
    res.z= ptr[8]*p.x +ptr[9]*p.y +ptr[10]*p.z+ptr[11];
    return res;
}



/**
    * Multiplication of m by p. m is a 4x4 matrix
    */
template < typename Point3dType>
Point3dType mult ( const cv::Mat &m,  Point3dType  p ) {
    assert ( m.isContinuous() );
    if ( m.type() ==CV_32F ) {
        const float *ptr=m.ptr<float> ( 0 );
        Point3dType res;
        res.x= ptr[0]*p.x +ptr[1]*p.y +ptr[2]*p.z+ptr[3];
        res.y= ptr[4]*p.x +ptr[5]*p.y +ptr[6]*p.z+ptr[7];
        res.z= ptr[8]*p.x +ptr[9]*p.y +ptr[10]*p.z+ptr[11];
        return res;
    } else 	if ( m.type() ==CV_64F ) {
        const double *ptr=m.ptr<double> ( 0 );
        Point3dType res;
        res.x= ptr[0]*p.x +ptr[1]*p.y +ptr[2]*p.z+ptr[3];
        res.y= ptr[4]*p.x +ptr[5]*p.y +ptr[6]*p.z+ptr[7];
        res.z= ptr[8]*p.x +ptr[9]*p.y +ptr[10]*p.z+ptr[11];
        return res;

    }
    return Point3dType();
}

template < typename Point3dType>
void mult ( const cv::Mat &m,  std::vector<Point3dType > & vp ) {
    assert ( m.isContinuous() );
    if ( m.type() ==CV_32F ) {
        const float *ptr=m.ptr<float> ( 0 );
        Point3dType res;
        for ( auto & p :  vp ) {
            res.x= ptr[0]*p.x +ptr[1]*p.y +ptr[2]*p.z+ptr[3];
            res.y= ptr[4]*p.x +ptr[5]*p.y +ptr[6]*p.z+ptr[7];
            res.z= ptr[8]*p.x +ptr[9]*p.y +ptr[10]*p.z+ptr[11];
            p=res;
        }
    } else 	if ( m.type() ==CV_64F ) {
        const double *ptr=m.ptr<double> ( 0 );
        Point3dType res;
        for ( auto & p :  vp ) {
            res.x= ptr[0]*p.x +ptr[1]*p.y +ptr[2]*p.z+ptr[3];
            res.y= ptr[4]*p.x +ptr[5]*p.y +ptr[6]*p.z+ptr[7];
            res.z= ptr[8]*p.x +ptr[9]*p.y +ptr[10]*p.z+ptr[11];
            p=res;
        }
    }
}

//calculates the reprojection error in only one camera
MARKERMAPPER_API  double getReprjError ( const   std::vector<cv::Point3f>   &objectPoints,const   std::vector<cv::Point2f>  &points1,cv::Mat cam1,cv::Mat dist1, cv::Mat vT1 );
MARKERMAPPER_API  cv::Mat getRTMatrix ( const cv::Mat &R,const cv::Mat &T  ,int forceType=-1 );
MARKERMAPPER_API  void getRTfromMatrix44 ( const cv::Mat &M,  cv::Mat &R,cv::Mat &T );
MARKERMAPPER_API  void getInverseRT(   cv::Mat &R,  cv::Mat &T  );



/**Rigid transform between a set of points using horn
     */
MARKERMAPPER_API float rigidBodyTransformation_Horn1987 ( cv::Mat& orgPoints_32FC3, cv::Mat& dstPoints_32FC3,cv::Mat &RT_4x4 );
MARKERMAPPER_API float rigidBodyTransformation_Horn1987 ( cv::Mat& orgPoints_32FC3, cv::Mat& dstPoints_32FC3,cv::Mat &Rvec,cv::Mat &Tvec );
MARKERMAPPER_API float rigidBodyTransformation_Horn1987 (const std::vector<cv::Point3f> &orgPoints_32FC3, const std::vector<cv::Point3f> &dstPoints_32FC3, cv::Mat &Rvec, cv::Mat &Tvec );
MARKERMAPPER_API cv::Mat rigidBodyTransformation_Horn1987 (const std::vector<cv::Point3f> &orgPoints_32FC3, const std::vector<cv::Point3f> &dstPoints_32FC3,double *err=0);


//returns the 3d location (relative to camera of a point in the depth maps)
//if the point has no valid depth, z component is 0
cv::Point3f  get3d ( cv::Point p, const cv::Mat & rangeMap, cv::Mat  cam, int wsize=0 ) ;


cv::Mat  getMatrixFromQuaternion(double qx,double qy, double qz,double qw,double tx,double ty ,double tz);
void getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,double &qx,double &qy,double &qz,double &qw,double &tx,double &ty,double &tz);
void getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,float &qx,float &qy,float &qz,float &qw,float &tx,float &ty,float &tz);

std::vector<cv::Point2f> projectPoints(const std::vector<cv::Point3f> &p3d,   se3 &pose ,const aruco::CameraParameters &cp);

}

#endif
