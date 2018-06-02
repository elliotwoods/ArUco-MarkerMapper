#ifndef MatUtils_HPP_
#define MatUtils_HPP_

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <exception>
namespace _3dutils {

void savePCDFile(const char *filename,  const std::vector<cv::Point3f> &points,  const  cv::Scalar color ) ;



//calculates the reprojection error in only one camera
double getReprjError ( const   std::vector<cv::Point3f>   &objectPoints,const   std::vector<cv::Point2f>  &points1,cv::Mat cam1,cv::Mat dist1, cv::Mat vT1 );
cv::Mat getRTMatrix ( const cv::Mat &R,const cv::Mat &T  ,int forceType=-1 );
void getRTfromMatrix44 ( const cv::Mat &M,  cv::Mat &R,cv::Mat &T );
void getInverseRT(   cv::Mat &R,  cv::Mat &T  );


//triangulate and returns true if valid triangulation

void triangulate ( const cv::Mat &points0, const cv::Mat & camMatrix0,const cv::Mat &distCoef0,const cv::Mat &T0_,
                   const cv::Mat &points1, const cv::Mat & camMatrix1,const cv::Mat &distCoef1,const cv::Mat &T1_,
                   cv::Mat &objectPoints,  std::vector<uchar> & status );
/**Rigid transform between a set of points using horn
     */
float rigidBodyTransformation_Horn1987 ( cv::Mat& orgPoints_32FC3, cv::Mat& dstPoints_32FC3,cv::Mat &RT_4x4 );
float rigidBodyTransformation_Horn1987 ( cv::Mat& orgPoints_32FC3, cv::Mat& dstPoints_32FC3,cv::Mat &Rvec,cv::Mat &Tvec );
float rigidBodyTransformation_Horn1987 (const std::vector<cv::Point3f> &orgPoints_32FC3, const std::vector<cv::Point3f> &dstPoints_32FC3, cv::Mat &Rvec, cv::Mat &Tvec );
cv::Mat rigidBodyTransformation_Horn1987 (const std::vector<cv::Point3f> &orgPoints_32FC3, const std::vector<cv::Point3f> &dstPoints_32FC3,double *err=0);


cv::Mat  getMatrixFromQuaternion(double qx,double qy, double qz,double qw,double tx,double ty ,double tz);
void getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,double &qx,double &qy,double &qz,double &qw,double &tx,double &ty,double &tz);
void getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,float &qx,float &qy,float &qz,float &qw,float &tx,float &ty,float &tz);

/**Fast undistortion of a point
 * If normalize is true, points are in range [0,1]
    */
cv::Point2f undistortPoint ( cv::Mat  camMatrix,cv::Mat  distCoeff,cv::Point2f p ,bool normalize=true);


/**Transform of a 3f point
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

}

#endif
