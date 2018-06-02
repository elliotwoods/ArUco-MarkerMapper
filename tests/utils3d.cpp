#include "utils3d.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
using namespace std;
namespace _3dutils {

/**
  * Calculate the line segment PaPb that is the shortest route between
  * two lines P1P2 and P3P4. Calculate also the values of mua and mub where
  *    Pa = P1 + mua (P2 - P1)
  *    Pb = P3 + mub (P4 - P3)
  * Return FALSE if no solution exists.
*/
bool  lineLineIntersect (
    cv::Point3d p1,cv::Point3d p2,cv::Point3d p3,cv::Point3d p4,cv::Point3d *pa,cv::Point3d *pb,
    double *mua, double *mub )
{
    double EPS=1e-4;
    cv::Point3d p13,p43,p21;
    double d1343,d4321,d1321,d4343,d2121;
    double numer,denom;

    p13.x = p1.x - p3.x;
    p13.y = p1.y - p3.y;
    p13.z = p1.z - p3.z;
    p43.x = p4.x - p3.x;
    p43.y = p4.y - p3.y;
    p43.z = p4.z - p3.z;
    if ( fabs ( p43.x )  < EPS && fabs ( p43.y )  < EPS && fabs ( p43.z )  < EPS )
        return false;
    p21.x = p2.x - p1.x;
    p21.y = p2.y - p1.y;
    p21.z = p2.z - p1.z;
    if ( fabs ( p21.x )  < EPS && fabs ( p21.y )  < EPS && fabs ( p21.z )  < EPS )
        return false;

    d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
    d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
    d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
    d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
    d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

    denom = d2121 * d4343 - d4321 * d4321;
    if ( fabs ( denom ) < EPS )
        return false;
    numer = d1343 * d4321 - d1321 * d4343;

    *mua = numer / denom;
    *mub = ( d1343 + d4321 * ( *mua ) ) / d4343;

    pa->x = p1.x + *mua * p21.x;
    pa->y = p1.y + *mua * p21.y;
    pa->z = p1.z + *mua * p21.z;
    pb->x = p3.x + *mub * p43.x;
    pb->y = p3.y + *mub * p43.y;
    pb->z = p3.z + *mub * p43.z;

    return true;
}

/**
 * Basic triangulation
 *
 */
void triangulate ( const cv::Mat &points0, const cv::Mat & camMatrix0,const cv::Mat &distCoef0,const cv::Mat &T0_,
                   const cv::Mat &points1, const cv::Mat & camMatrix1,const cv::Mat &distCoef1,const cv::Mat &T1_,
                   cv::Mat &objectPoints,  vector<uchar> & status )

{
     assert ( points0.cols==2 || points0.channels() ==2 );
    assert ( points0.size() ==points1.size() );
    cv::Mat CamMatrix[2];
    camMatrix0.convertTo ( CamMatrix[0],CV_32F );
    camMatrix1.convertTo ( CamMatrix[1],CV_32F );

    cv::Mat T0,T1;
    T0_.convertTo ( T0,CV_32F );
    T1_.convertTo ( T1,CV_32F );

    cv::Mat undP0,undP1;
    cv::undistortPoints ( points0,undP0 ,CamMatrix[0],distCoef0);
    cv::undistortPoints ( points1,undP1 ,CamMatrix[1],distCoef1);

    //calculate the line between the camera centers and the projection
    cv::Mat M0inv=T0.inv();
    cv::Mat M1inv=T1.inv();
    float cm_fx[2],cm_fy[2],cm_cx[2],cm_cy[2];
    for ( int i=0;i<2;i++ )
    {
        cm_fx[i]=CamMatrix[i].at<float> ( 0,0 );
        cm_fy[i]=CamMatrix[i].at<float> ( 1,1 );
        cm_cx[i]=CamMatrix[i].at<float> ( 0,2 );
        cm_cy[i]=CamMatrix[i].at<float> ( 1,2 );
    }


    pair< cv::Point3f,cv::Point3f >  Line0,Line1;
    //first point of the line is the camera centre. transform it to the global coordinate system
    Line0.first=cv::Point3f ( 0,0,0 );
    Line0.first=mult<float> ( M0inv,Line0.first );
    //first point of the line is the camera centre. transform it to the global coordinate system
    Line1.first=cv::Point3f ( 0,0,0 );
    Line1.first=mult<float> ( M1inv,Line1.first );

    int nPoints=points0.rows;
    status.resize ( nPoints );
    objectPoints.create ( nPoints,3,CV_32FC1 );
    float *xyz=objectPoints.ptr<float> ( 0 );
    assert ( undP0.isContinuous() & undP1.isContinuous() );
    float *xy0=undP0.ptr<float> ( 0 );
    float *xy1=undP1.ptr<float> ( 0 );

    for ( int i=0; i< nPoints; i++,xyz+=3 )
    {

        cv::Point2f imgPoint0 ( xy0[i*2],xy0[i*2+1] );
        cv::Point2f imgPoint1 ( xy1[i*2],xy1[i*2+1] );

        //second point in the lines is the one passing through the projected point at a distance of z=1
        Line0.second.x= xy0[i*2];//( imgPoint0.x-cm_cx[0] ) /cm_fx[0];
        Line0.second.y=xy0[i*2+1];// ( imgPoint0.y-cm_cy[0] ) /cm_fy[0];
        Line0.second.z=1;
        Line0.second=mult<float> ( M0inv,Line0.second );

        Line1.second.x=xy1[i*2];// ( imgPoint1.x-cm_cx[1] ) /cm_fx[1];
        Line1.second.y= xy1[i*2+1];//( imgPoint1.y-cm_cy[1] ) /cm_fy[1];
        Line1.second.z=1;
        Line1.second=mult<float> ( M1inv,Line1.second );
        //now, find the intersection
        cv::Point3d p1,p2;
        double mua,mub;

        if ( lineLineIntersect ( Line0.first,Line0.second,Line1.first,Line1.second,&p1,&p2,&mua,&mub ) )
        {
            xyz[0]= ( p1.x+p2.x ) * 0.5;
            xyz[1]= ( p1.y+p2.y ) * 0.5;
            xyz[2]= ( p1.z+p2.z ) * 0.5;
            status[i]=1;
        }
        else
        {
            status[i]=0;
        }
    }
}

void savePCDFile(const char *filename,  const std::vector<cv::Point3f> &points,
                 const  cv::Scalar color ) {

    int num_points = points.size();

    std::ofstream filePCD ( filename, std::ios::binary );

    filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<num_points<<"\nHEIGHT 1\nPOINTS "<<num_points<<"\nDATA binary\n";


    unsigned char ucolor[4]={uchar(color.val[0]), uchar(color.val[1]),uchar(color.val[2]),uchar(0)};

    for ( size_t i=0; i<num_points; i++ ){
        float coord[3];
        coord[0] = points[i].x;
        coord[1] = points[i].y;
        coord[2] = points[i].z;

        filePCD.write ( ( char* ) &(coord[0]), 3*sizeof ( float ) );
        //now, color


        filePCD.write ( ( char* ) &(ucolor[0]), 4*sizeof ( unsigned char ) );
    }

    filePCD.close();
}

/**
       * Given a Rotation and a Translation expressed both as a vector, returns the corresponding 4x4 matrix
       */
cv::Mat getRTMatrix ( const cv::Mat &R_,const cv::Mat &T_ ,int forceType ) {
    cv::Mat M;
    cv::Mat R,T;
    R_.copyTo ( R );
    T_.copyTo ( T );
    if ( R.type() ==CV_64F ) {
        assert ( T.type() ==CV_64F );
        cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_64FC1 );

        cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
        if ( R.total() ==3 ) {
            cv::Rodrigues ( R,R33 );
        } else if ( R.total() ==9 ) {
            cv::Mat R64;
            R.convertTo ( R64,CV_64F );
            R.copyTo ( R33 );
        }
        for ( int i=0; i<3; i++ )
            Matrix.at<double> ( i,3 ) =T.ptr<double> ( 0 ) [i];
        M=Matrix;
    } else if ( R.depth() ==CV_32F ) {
        cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_32FC1 );
        cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
        if ( R.total() ==3 ) {
            cv::Rodrigues ( R,R33 );
        } else if ( R.total() ==9 ) {
            cv::Mat R32;
            R.convertTo ( R32,CV_32F );
            R.copyTo ( R33 );
        }

        for ( int i=0; i<3; i++ )
            Matrix.at<float> ( i,3 ) =T.ptr<float> ( 0 ) [i];
        M=Matrix;
    }

    if ( forceType==-1 ) return M;
    else {
        cv::Mat MTyped;
        M.convertTo ( MTyped,forceType );
        return MTyped;
    }
}

double getReprjError ( const  vector<vector<cv::Point3f> > &objectPoints,const vector< vector<cv::Point2f> >&points1,cv::Mat cam1,cv::Mat dist1,vector<cv::Mat>vT1
                       ) {
    double  errReprj=0,n=0;
    double minE=std::numeric_limits<double>::max(),maxE=std::numeric_limits<double>::min();
    for ( size_t i=0; i<points1.size(); i++ ) {
        //reproject points
        //move 3d points and project from 0
        cv::Mat  R,T;
        vector<cv::Point2f> pointsr1;
        getRTfromMatrix44 ( vT1[i],R,T );
        cv::projectPoints ( objectPoints[i] , R,T,cam1,dist1,pointsr1 );
        for ( size_t j=0; j<points1[i].size(); j++ ) {
            double err=cv::norm ( points1[i][j]- pointsr1[j] );;
            errReprj+= err;
            minE=std::min ( minE,err );
            maxE=std::max ( maxE,err );
            n++;
        }
    }
    //         cout<<"GET REP ERROR="<<errReprj/ double ( n )<<" min max="<<minE<<" "<<maxE<<endl;
    return errReprj/ double ( n );
}


void getRTfromMatrix44 ( const cv::Mat &M,  cv::Mat &R,cv::Mat &T ) {

    assert ( M.cols==M.rows && M.cols==4 );
    assert ( M.type() ==CV_32F || M.type() ==CV_64F );
    //extract the rotation part
    cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
    cv::SVD svd ( r33 );
    cv::Mat Rpure=svd.u*svd.vt;
    cv::Rodrigues ( Rpure,R );
    T.create ( 1,3,M.type() );
    if ( M.type() ==CV_32F )
        for ( int i=0; i<3; i++ )
            T.ptr<float> ( 0 ) [i]=M.at<float> ( i,3 );
    else
        for ( int i=0; i<3; i++ )
            T.ptr<double> ( 0 ) [i]=M.at<double> ( i,3 );
}

/**********************
     *
     *
     **********************/
struct Quaternion
{
    Quaternion ( float q0,float q1,float q2,float q3 ) {
        q[0]=q0;
        q[1]=q1;
        q[2]=q2;
        q[3]=q3;
    }
    cv::Mat getRotation() const
    {
        cv::Mat R ( 3,3,CV_32F );
        R.at<float> ( 0,0 ) =q[0]*q[0] +q[1]*q[1] -q[2]*q[2] -q[3]*q[3];
        R.at<float> ( 0,1 ) =2.* ( q[1]*q[2] - q[0]*q[3] );
        R.at<float> ( 0,2 ) =2.* ( q[1]*q[3] + q[0]*q[2] );

        R.at<float> ( 1,0 ) =2.* ( q[1]*q[2] + q[0]*q[3] );
        R.at<float> ( 1,1 ) =q[0]*q[0] +q[2]*q[2] -q[1]*q[1] -q[3]*q[3];
        R.at<float> ( 1,2 ) =2.* ( q[2]*q[3] - q[0]*q[1] );

        R.at<float> ( 2,0 ) =2.* ( q[1]*q[3] - q[0]*q[2] );
        R.at<float> ( 2,1 ) =2.* ( q[2]*q[3] + q[0]*q[1] );
        R.at<float> ( 2,2 ) =q[0]*q[0] +q[3]*q[3] -q[1]*q[1] -q[2]*q[2];
        return R;
    }
    float q[4];
};

/**********************
     *
     *
     **********************/
float rigidBodyTransformation_Horn1987 ( cv::Mat& S, cv::Mat& M,cv::Mat &RT_4x4 ) {

    assert ( S.total() ==M.total() );
    assert ( S.type() ==M.type() );
    assert ( S.rows>S.cols && M.rows>M.cols );

    cv::Mat _s,_m;
    S.convertTo ( _s,CV_32F );
    M.convertTo ( _m,CV_32F );
    _s=_s.reshape ( 1 );
    _m=_m.reshape ( 1 );
    cv::Mat Mu_s=cv::Mat::zeros ( 1,3,CV_32F );
    cv::Mat Mu_m=cv::Mat::zeros ( 1,3,CV_32F );
    //         cout<<_s<<endl<<_m<<endl;
    //calculate means
    for ( int i=0; i<_s.rows; i++ ) {
        Mu_s+=_s ( cv::Range ( i,i+1 ),cv::Range ( 0,3 ) );
        Mu_m+=_m ( cv::Range ( i,i+1 ),cv::Range ( 0,3 ) );
    }
    //now, divide
    for ( int i=0; i<3; i++ ) {
        Mu_s.ptr<float> ( 0 ) [i]/=float ( _s.rows );
        Mu_m.ptr<float> ( 0 ) [i]/=float ( _m.rows );
    }

    // cout<<"Mu_s="<<Mu_s<<endl;
    // cout<<"Mu_m="<<Mu_m<<endl;

    cv::Mat Mu_st=Mu_s.t() *Mu_m;
    // cout<<"Mu_st="<<Mu_st<<endl;
    cv::Mat Var_sm=cv::Mat::zeros ( 3,3,CV_32F );
    for ( int i=0; i<_s.rows; i++ )
        Var_sm+= ( _s ( cv::Range ( i,i+1 ),cv::Range ( 0,3 ) ).t() *_m ( cv::Range ( i,i+1 ),cv::Range ( 0,3 ) ) ) -  Mu_st;
    //   cout<<"Var_sm="<<Var_sm<<endl;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ )
            Var_sm.at<float> ( i,j ) /=float ( _s.rows );
    //   cout<<"Var_sm="<<Var_sm<<endl;

    cv::Mat AA=Var_sm-Var_sm.t();
    //     cout<<"AA="<<AA<<endl;
    cv::Mat A ( 3,1,CV_32F );
    A.at<float> ( 0,0 ) =AA.at<float> ( 1,2 );
    A.at<float> ( 1,0 ) =AA.at<float> ( 2,0 );
    A.at<float> ( 2,0 ) =AA.at<float> ( 0,1 );
    //     cout<<"A ="<<A <<endl;
    cv::Mat Q_Var_sm ( 4,4,CV_32F );
    Q_Var_sm.at<float> ( 0,0 ) =trace ( Var_sm ) [0];
    for ( int i=1; i<4; i++ ) {
        Q_Var_sm.at<float> ( 0,i ) =A.ptr<float> ( 0 ) [i-1];
        Q_Var_sm.at<float> ( i,0 ) =A.ptr<float> ( 0 ) [i-1];
    }
    cv::Mat q33=Var_sm+Var_sm.t()- ( trace ( Var_sm ) [0]*cv::Mat::eye ( 3,3,CV_32F ) );

    cv::Mat Q33=Q_Var_sm ( cv::Range ( 1,4 ),cv::Range ( 1,4 ) );
    q33.copyTo ( Q33 );
    // cout<<"Q_Var_sm"<<endl<< Q_Var_sm<<endl;
    cv::Mat eigenvalues,eigenvectors;
    eigen ( Q_Var_sm,eigenvalues,eigenvectors );
    // cout<<"EEI="<<eigenvalues<<endl;
    // cout<<"V="<<(eigenvectors.type()==CV_32F)<<" "<<eigenvectors<<endl;

    Quaternion rot ( eigenvectors.at<float> ( 0,0 ),eigenvectors.at<float> ( 0,1 ),eigenvectors.at<float> ( 0,2 ),eigenvectors.at<float> ( 0,3 ) );
    cv::Mat RR=rot.getRotation();
    //  cout<<"RESULT="<<endl<<RR<<endl;
    cv::Mat T= Mu_m.t()- RR*Mu_s.t();
    //  cout<<"T="<<T<<endl;

    RT_4x4=cv::Mat::eye ( 4,4,CV_32F );
    cv::Mat r33=RT_4x4 ( cv::Range ( 0,3 ),cv::Range ( 0,3 ) );
    RR.copyTo ( r33 );
    for ( int i=0; i<3; i++ ) RT_4x4.at<float> ( i,3 ) =T.ptr<float> ( 0 ) [i];
    //  cout<<"RESS="<<RT<<endl;

    //compute the average transform error

    float err=0;
    float *matrix=RT_4x4.ptr<float> ( 0 );
    for ( size_t i=0; i<S.rows; i++ ) {
        cv::Point3f org= S.ptr<cv::Point3f> ( 0 ) [i];
        cv::Point3f dest_est;
        dest_est.x= matrix[0]*org.x+ matrix[1]*org.y +matrix[2]*org.z+matrix[3];
        dest_est.y= matrix[4]*org.x+ matrix[5]*org.y +matrix[6]*org.z+matrix[7];
        dest_est.z= matrix[8]*org.x+ matrix[9]*org.y +matrix[10]*org.z+matrix[11];
        cv::Point3f dest_real=M.ptr<cv::Point3f> ( 0 ) [ i ];
        err+= cv::norm ( dest_est-dest_real )*cv::norm ( dest_est-dest_real );
    }
    return sqrt(err/float ( S.rows ));
}
/**********************
     *
     *
     **********************/
float rigidBodyTransformation_Horn1987 ( cv::Mat& _s, cv::Mat& _m,cv::Mat &Rvec,cv::Mat &Tvec ) {
    cv::Mat RT;
    float err= rigidBodyTransformation_Horn1987 ( _s,_m,RT );
    getRTfromMatrix44 ( RT,Rvec,Tvec );
    return err;
}


float rigidBodyTransformation_Horn1987 (const vector<cv::Point3f> & orgPoints_32FC3,const vector<cv::Point3f> &dstPoints_32FC3,cv::Mat &Rvec,cv::Mat &Tvec ) {
    cv::Mat Morg,Mdest;
    Morg.create ( orgPoints_32FC3.size(),1,CV_32FC3 );
    Mdest.create ( dstPoints_32FC3.size(),1,CV_32FC3 );
    for ( size_t i=0; i<dstPoints_32FC3.size(); i++ ) {
        Morg.ptr<  cv::Point3f> ( 0 ) [i]=orgPoints_32FC3[i];
        Mdest.ptr<  cv::Point3f> ( 0 ) [i]=dstPoints_32FC3[i];
    }
    return rigidBodyTransformation_Horn1987 ( Morg,Mdest,Rvec,Tvec );

}


cv::Mat rigidBodyTransformation_Horn1987 (const std::vector<cv::Point3f> &org, const std::vector<cv::Point3f> &dst,double *err){
    double e=0;
    cv::Mat r,t;
    e=rigidBodyTransformation_Horn1987(org,dst,r,t);
    if (err) *err=e;
    return getRTMatrix(r,t);
}



cv::Mat  getMatrixFromQuaternion(double qx,double qy, double qz,double qw,double tx,double ty ,double tz){

    Eigen::Quaternionf q( qw,qx,qy,qz);

    auto m3=q.toRotationMatrix ();
    cv::Mat m=cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            m.at<float>(i,j)=m3(i,j);
    m.at<float>(0,3)=tx;
    m.at<float>(1,3)=ty;
    m.at<float>(2,3)=tz;
    return m;
}
void getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,double &qx,double &qy,double &qz,double &qw,double &tx,double &ty,double &tz){

    float fqx,fqy,fqz,fqw,ftx,fty,ftz;
    getQuaternionAndTranslationfromMatrix44(M_in,fqx,fqy,fqz,fqw,ftx,fty,ftz);
    qx=fqx;
    qy=fqy;
    qz=fqz;
    qw=fqw;
    tx=ftx;
    ty=fty;
    tz=ftz;
}

void getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,float &qx,float &qy,float &qz,float &qw,float &tx,float &ty,float &tz){
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


template<class T>
cv::Point2f undistortPoint_T ( cv::Mat  camMatrix,cv::Mat  distCoeff,cv::Point2f p, bool normalize ) {

    assert ( distCoeff.total() ==5 );

    T *k=distCoeff.ptr<T> ( 0 );

    float x0 = p.x = ( p.x - camMatrix.at<T> ( 0,2 ) ) / camMatrix.at<T> ( 0,0 );
    float y0 = p.y = ( p.y - camMatrix.at<T> ( 1,2 ) ) /camMatrix.at<T> ( 1,1 );

    // compensate distortion iteratively
    for ( int j = 0; j < 5; j++ ) {
        T r2 = p.x*p.x + p.y*p.y;
        T icdist = 1 / ( 1 + ( ( k[4]*r2 + k[1] ) *r2 + k[0] ) *r2 );
        T deltaX = 2*k[2]*p.x*p.y + k[3]* ( r2 + 2*p.x*p.x );
        T deltaY = k[2]* ( r2 + 2*p.y*p.y ) + 2*k[3]*p.x*p.y;
        p.x = ( x0 - deltaX ) *icdist;
        p.y = ( y0 - deltaY ) *icdist;
    }
    if ( !normalize ) {
        p.x =   p.x*camMatrix.at<T> ( 0,0 ) + camMatrix.at<T> ( 0,2 )  ;
        p.y =  p.y *camMatrix.at<T> ( 1,1 ) +camMatrix.at<T> ( 1,2 ) ;
    }
    return p;
}


    cv::Point2f undistortPoint ( cv::Mat  camMatrix,cv::Mat  distCoeff,cv::Point2f p, bool normalize ) {
        if (distCoeff.type()==CV_64F) return undistortPoint_T<double>(camMatrix,distCoeff,p,normalize);
    if (distCoeff.type()==CV_32F) return undistortPoint_T<float>(camMatrix,distCoeff,p,normalize);
    }
}
