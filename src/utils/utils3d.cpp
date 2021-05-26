#include "utils3d.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "debug.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "se3.h"
using namespace std;
namespace aruco_mm {
float getColor(cv::Scalar color)
{
    float fcolor;uchar *c=(uchar*)&fcolor;
    for(int i=0;i<3;i++)c[i]=color[i];
    return fcolor;
}

std::vector<cv::Vec4f> getPcdPoints(const vector<cv::Point3f> &mpoints,cv::Scalar color,int npoints=100){
   vector<cv::Vec4f> points;
   double msize=cv::norm(mpoints[0]-mpoints[1]);
   float fcolor;uchar *c=(uchar*)&fcolor;
   for(int i=0;i<3;i++)c[i]=color[i];

   //lines joining points
   for(size_t i=0;i<mpoints.size();i++){
       cv::Point3f v=mpoints[(i+1)%mpoints.size()]-mpoints[i];
       float ax=1./float(npoints);//npoints
       for(float x=0;x<=1;x+=ax){
           cv::Point3f p3=mpoints[i]+v*x;
           points.push_back(cv::Vec4f(p3.x,p3.y,p3.z, fcolor));
       }
   }

   //line indicating direction
   //take first and second, first and last , and get the cross vector indicating the direction
   cv::Point3f v1=mpoints[1]-mpoints[0];
   cv::Point3f v2=mpoints[3]-mpoints[0];
    v1*=1./cv::norm(v1);
   v2*=1./cv::norm(v2);
   cv::Point3f vz=v2.cross(v1);
   vz*=1./cv::norm(vz);//now, unit

   //set the center
   cv::Point3f center=(mpoints[0]+mpoints[1]+mpoints[2]+mpoints[3])*0.25;
   float ax=(msize/3)/100;
   for(float x=0;x<=msize/3;x+=ax){
       cv::Point3f p3=center+vz*x;
       points.push_back(cv::Vec4f(p3.x,p3.y,p3.z, getColor(cv::Scalar(255,255,255)-color)));

   }


//   for(float x=0;x<=2*msize/3;x+=ax){
//       cv::Point3f p3=center+v1*x;
//       points.push_back(cv::Vec4f(p3.x,p3.y,p3.z,getColor(cv::Scalar(0,255,0))));

//   }
//   for(float x=0;x<=2*msize/3;x+=ax){
//       cv::Point3f p3=center+v2*x;
//       points.push_back(cv::Vec4f(p3.x,p3.y,p3.z, getColor(cv::Scalar(255,0,0))));

//   }
   return points;

}

vector<cv::Vec4f> getMarkerIdPcd(int id,float markerSize,cv::Mat rt_g2m,cv::Scalar color ){
   //marker id as a set of points
       auto _to_string=[](int i){ 	std::stringstream str;str<<i;return str.str(); };
   string text = _to_string(id);
   int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
   double fontScale = 2;
   int thickness = 3;
   int baseline=0;
   float markerSize_2=markerSize/2;
   cv::Size textSize = cv::getTextSize(text, fontFace,
                               fontScale, thickness, &baseline);
   cv::Mat img(textSize +cv::Size(0,baseline/2), CV_8UC1,cv::Scalar::all(0));
   // center the text
   // then put the text itself
   cv::putText(img, text, cv::Point(0,textSize.height+baseline/4), fontFace, fontScale,cv::Scalar::all(255), thickness, 8);
   //raster 2d points as 3d points
   vector<cv::Point3f> points_id;
   for(int y=0;y<img.rows;y++)
       for(int x=0;x<img.cols;x++)
           if (img.at<uchar>(y,x)!=0) points_id.push_back( cv::Point3f((float(x)/float(img.cols))-0.5,(float(img.rows-y)/float(img.rows))-0.5,0));


   //now,scale
   for(auto &p:points_id)p*=markerSize_2;
   //finally, translate
   for(auto &p:points_id)p=aruco_mm::mult<float>( rt_g2m,p);
   //now, add to ouput

   float fcolor;uchar *c=(uchar*)&fcolor;
   for(int i=0;i<3;i++)c[i]=color[i];

    vector<cv::Vec4f> res;
   for(auto &p:points_id)
       res.push_back(cv::Vec4f(p.x,p.y,p.z, fcolor));



   return res;
}
std::vector<cv::Vec4f> getLine(const  cv::Point3f &a,const  cv::Point3f &b,cv::Scalar color,int npoints){
    float fcolor;uchar *c=(uchar*)&fcolor;
    for(int i=0;i<3;i++)c[i]=color[i];

    std::vector<cv::Vec4f>  points;
    cv::Vec4f pa(a.x,a.y,a.z,fcolor);
    cv::Vec4f pb(b.x,b.y,b.z,fcolor);
    cv::Vec4f d =pb-pa;

    double step= cv::norm(pa-pb) / double(npoints);
    for(int i=0;i<npoints;i++)
        points.push_back(pa+ (d*step*double(i)));
    return points;

}

std::vector<cv::Vec4f>  getCameraPoints(const cv::Mat &rt_c2g,float size){
   //write base as a cube
   cv::Mat g2c=rt_c2g.inv();

   cv::Point3f(0,0,0);
   std::vector<cv::Vec4f>   points;
   double size_2=size/2.;
   auto paxis=getLine( mult(g2c,cv::Point3f(-size_2,size_2,0)),mult(g2c,cv::Point3f(size_2,size_2,0)),cv::Scalar(0,255,255),50);
   points.insert(points.end(),paxis.begin(),paxis.end());
   paxis=getLine( mult(g2c,cv::Point3f(size_2,size_2,0)),mult(g2c,cv::Point3f(size_2,-size_2,0)),cv::Scalar(0,255,255),50);
//    points.insert(points.end(),paxis.begin(),paxis.end());
//    paxis=getLine( mult(g2c,cv::Point3f(size_2,-size_2,0)),mult(g2c,cv::Point3f(-size_2,-size_2,0)),cv::Scalar(0,255,255),50);
//    points.insert(points.end(),paxis.begin(),paxis.end());
//    paxis=getLine( mult(g2c,cv::Point3f(-size_2,-size_2,0)),mult(g2c,cv::Point3f(-size_2,size_2,0)),cv::Scalar(0,255,255),50);
//    points.insert(points.end(),paxis.begin(),paxis.end());

   paxis=getLine( mult(g2c,cv::Point3f(0,0,0)),mult(g2c,cv::Point3f(0,0,size)),cv::Scalar(255,0,0),50);
           points.insert(points.end(),paxis.begin(),paxis.end());

   return points;
}


void  savePCDFile(string fpath, const MarkerSet &ms, int origin) {
    (void)origin;
    std::vector<cv::Vec4f> points2write;
    for(auto m:ms)
    {
        cv::Scalar color;
        color=cv::Scalar(255,0,0);
        auto points4=getPcdPoints(getMarkerPoints(m.second.markerSize,m.second.rt_g2m),color);
        points2write.insert(points2write.end(),points4.begin(),points4.end());
        auto points_id=getMarkerIdPcd(m.second.id,m.second.markerSize,m.second.rt_g2m,color);
        points2write.insert(points2write.end(),points_id.begin(),points_id.end());
    }




    std::ofstream filePCD ( fpath, std::ios::binary );
    filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points2write.size()<<"\nHEIGHT 1\nPOINTS "<<points2write.size()<<"\nDATA binary\n";


    filePCD.write((char*)&points2write[0],points2write.size()*sizeof(points2write[0]));



}

void savePCDFile(string fpath,const MarkerSet &ms, const vector<cv::Mat >  &vector_rt_c2g,int org_id ) {
    std::vector<cv::Vec4f> points2write;
    float max_msize=-1;
    for(auto m:ms)
    {
        cv::Scalar color;
        if (m.second.id==org_id)color=cv::Scalar(0,0,255);
        else color=cv::Scalar(255,0,0);
        auto points4=getPcdPoints(getMarkerPoints(m.second.markerSize,m.second.rt_g2m),color);
        points2write.insert(points2write.end(),points4.begin(),points4.end());
        auto points_id=getMarkerIdPcd(m.second.id,m.second.markerSize,m.second.rt_g2m,color);
        points2write.insert(points2write.end(),points_id.begin(),points_id.end());
        max_msize=std::max(max_msize,m.second.markerSize);
    }
    for(auto rt_c2g:vector_rt_c2g){
        if (!rt_c2g.empty()){
            cv::Mat g2c=rt_c2g.inv();
            auto pcam=getPcdPoints(getMarkerPoints(max_msize/2,g2c),cv::Scalar(0,255,0),25);
            points2write.insert(points2write.end(),pcam.begin(),pcam.end());

        }
    }


    std::ofstream filePCD ( fpath, std::ios::binary );

    filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points2write.size()<<"\nHEIGHT 1\nPOINTS "<<points2write.size()<<"\nDATA binary\n";


    filePCD.write((char*)&points2write[0],points2write.size()*sizeof(points2write[0]));

}

void savePCDFile(string fpath,const MarkerSet &ms, const FrameSet &fset,int org_id,bool writeFrameIds,cv::Scalar marker_color,cv::Scalar frame_color) {
    std::vector<cv::Vec4f> points2write;
    float max_msize=-1;
    for(auto m:ms)
    {
        cv::Scalar color;
        if (m.second.id==org_id)color=cv::Scalar(255,0,255)-marker_color;
        else color=marker_color;
        auto points4=getPcdPoints(getMarkerPoints(m.second.markerSize,m.second.rt_g2m),color);
        points2write.insert(points2write.end(),points4.begin(),points4.end());
        auto points_id=getMarkerIdPcd(m.second.id,m.second.markerSize,m.second.rt_g2m,color);
        points2write.insert(points2write.end(),points_id.begin(),points_id.end());
        max_msize=std::max(max_msize,m.second.markerSize);
    }
    for(auto frame:fset){
        if (!frame.second.rt_c2g.empty()){
            cv::Mat g2c=frame.second.rt_c2g.inv();
            auto pcam=getPcdPoints(getMarkerPoints(max_msize/2,g2c),frame_color,25);
            points2write.insert(points2write.end(),pcam.begin(),pcam.end());
          if (writeFrameIds){
              pcam=getMarkerIdPcd(frame.first,max_msize/2,frame.second.rt_c2g.inv(),cv::Scalar(255-frame_color[0],255-frame_color[1],255-frame_color[2]));//getCameraPoints(frame.second.rt_c2g,0.2);
            points2write.insert(points2write.end(),pcam.begin(),pcam.end());
          }
        }
    }


    std::ofstream filePCD ( fpath, std::ios::binary );

    filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points2write.size()<<"\nHEIGHT 1\nPOINTS "<<points2write.size()<<"\nDATA binary\n";


    filePCD.write((char*)&points2write[0],points2write.size()*sizeof(points2write[0]));

}




std::map<uint32_t,double>  getAvrgMarkerRepjError( const MarkerSet & ms,const FrameSet &fs,const aruco::CameraParameters &_cam_params){

    //aveg repj error per marker
    std::map<uint32_t,pair< int,double> > marker_err;
    for(auto m:ms)marker_err[m.first]=make_pair(0,0);
    for(auto f:fs){
        //  cout<<f.first<<" m->";

        if (!f.second.rt_c2g.empty()){
            for(auto m:f.second.markers){
                auto _ms=ms.find(m.id);
                if ( _ms!=ms.end()){
                    //    cout<<m.id<<"(";
                    cv::Mat r,t;aruco_mm::getRTfromMatrix44(f.second.rt_c2g,r,t);
                    vector<cv::Point2f> p2d;
                    cv::projectPoints(_ms->second.get3dPoints(),r,t,_cam_params.CameraMatrix,_cam_params.Distorsion,p2d);
                    for(size_t i=0;i<p2d.size();i++){
                        marker_err[ m.id].first++;
                        marker_err[ m.id].second+=cv::norm(p2d[i]-m[i]);
                        //        cout<<cv::norm(p2d[i]-m[i])<<" ";
                    }
                }
                //                cout<<")";
            }
        }
        //      else         cout<<" no";
        //    cout <<endl;
    }
    //now, compute average
    std::map<uint32_t,double>  res;
    for(auto &e:marker_err){
        e.second.second/=float(e.second.first);
        res[e.first]=e.second.second;
    }
    return res;
}

double   cameraMarkerDotProduct(cv::Mat c2m){
    cv::Point3f p0=aruco_mm::mult<float>(c2m,cv::Point3f(0,0,0)),p1=aruco_mm::mult<float>(c2m,cv::Point3f(0,0,1));
    cv::Point3f normal_=p1-p0;
    normal_*=1./cv::norm(normal_);
    return normal_.z;
}

std::vector<cv::Point3f>   getMarkerPoints( const MarkerInfo &m){
    return getMarkerPoints( m.markerSize,m.rt_g2m );
}

vector<cv::Point3f>   getMarkerPoints(float size,cv::Mat RT )
{
    float size_2=size/2.;
    //now, that the current location is estimated, add new markers and update old ones
    vector<cv::Point3f> points = { cv::Point3f ( -size_2, size_2,0 ),cv::Point3f ( size_2, size_2 ,0 ),
                                   cv::Point3f ( size_2, -size_2,0 ),cv::Point3f ( -size_2, -size_2,0 )  };
    if (!RT.empty())   aruco_mm::mult<cv::Point3f>(RT,points);

    return points;

}

cv::Mat  solvePnP(const vector<cv::Point3f> &p3d,const vector<cv::Point2f> &p2d,const aruco::CameraParameters &cp,bool ransac,cv::Mat initial_rt){
    cv::Mat r,t;
    if (!initial_rt.empty())        getRTfromMatrix44(initial_rt,r,t);
    if (ransac)
        cv::solvePnPRansac(p3d,p2d,cp.CameraMatrix,cp.Distorsion,r,t,!initial_rt.empty());
else
        cv::solvePnP(p3d,p2d,cp.CameraMatrix,cp.Distorsion,r,t,!initial_rt.empty());

    return aruco_mm::getRTMatrix(r,t,CV_32F);
}

cv::Mat  solvePnP(const  vector<aruco::Marker> &markers,    MarkerSet & MarkerSet,const aruco::CameraParameters &camp,bool ransac,cv::Mat initial_rt){
    vector<cv::Point2f> p2d;
    vector<cv::Point3f> p3d;
    for(size_t i=0;i<markers.size();i++){
        if ( MarkerSet.find(markers[i].id)!=MarkerSet.end()){
            p2d.insert(p2d.end(),markers[i].begin(),markers[i].end());
            auto m3d=getMarkerPoints(MarkerSet[markers[i].id].markerSize,MarkerSet[markers[i].id].rt_g2m);
            p3d.insert(p3d.end(),m3d.begin(),m3d.end());

        }
    }
    //solve 3d locaion
    //how many markers?
    cv::Mat r,t;
    if (!initial_rt.empty())        getRTfromMatrix44(initial_rt,r,t);
    if ( p3d.size()==4){//one, use rpp
        //return RPP::solveRpp(   p3d, p2d,_cam_params.CameraMatrix,_cam_params.Distorsion);
        cv::solvePnP(p3d,p2d,camp.CameraMatrix,camp.Distorsion,r,t,!initial_rt.empty());

    }
    else{
//        cout<<"ra="<<r<<" "<<t<<" "<<p3d.size()<<" "<<p2d.size()<<endl;
        if (ransac)
            cv::solvePnPRansac(p3d,p2d,camp.CameraMatrix,camp.Distorsion,r,t,!initial_rt.empty());
        else
            cv::solvePnP(p3d,p2d,camp.CameraMatrix,camp.Distorsion,r,t,!initial_rt.empty());
//        cout<<"rb="<<r<<" "<<t<<" "<<p3d.size()<<" "<<p2d.size()<<endl;
  //      cout<<"rp="<< getReprjError(p3d,p2d,camp.CameraMatrix,camp.Distorsion,aruco_mm::getRTMatrix(r,t,CV_32F))<<endl;

    }
    return aruco_mm::getRTMatrix(r,t,CV_32F);
}
void getInverseRT(   cv::Mat &R,  cv::Mat &T   ){

    cv::Mat rt=getRTMatrix(R,T);
    rt=rt.inv();
    getRTfromMatrix44(rt,R,T);
}


void savePCDFile(string fpath,const vector<cv::Point3f> &points){
    vector<cv::Vec4f> points2write;


    auto getfColor=[](uchar r,uchar g,uchar b)
    {
        float f;
        uchar *uf=(uchar*)&f;
        uf[0]=r;
        uf[1]=g;
        uf[2]=b;
        return f;
    };

    float colors[4]={getfColor(0,0,255),getfColor(0,255,0),getfColor(255,0,0),getfColor(125,125,125)};
    int idx=0;
    for(auto p:points) points2write.push_back(cv::Vec4f(p.x,p.y,p.z,colors[(idx++)%4]));
    std::ofstream filePCD ( fpath, std::ios::binary );
    filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points2write.size()<<"\nHEIGHT 1\nPOINTS "<<points2write.size()<<"\nDATA binary\n";

    filePCD.write((char*)&points2write[0],points2write.size()*sizeof(points2write[0]));



}

void savePCDFile(const char *filename,  const std::vector<cv::Point3f> &points,
                 const  cv::Scalar color ) {



    int num_points = points.size();

    std::ofstream filePCD ( filename, std::ios::binary );

    filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<num_points<<"\nHEIGHT 1\nPOINTS "<<num_points<<"\nDATA binary\n";


    unsigned char ucolor[4]={uchar(color.val[0]), uchar(color.val[1]),uchar(color.val[2]),uchar(0)};

    for ( int i=0; i<num_points; i++ ){
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
void savePCDFile(const char *filename,  const std::vector<cv::Point3f> &points,
                 const std::vector<cv::Scalar> &colors) {


    int num_points = points.size();

    std::ofstream filePCD ( filename, std::ios::binary );
    filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<num_points<<"\nHEIGHT 1\nPOINTS "<<num_points<<"\nDATA binary\n";


    for ( int i=0; i<num_points; i++ ){
      float coord[3];
      coord[0] = points[i].x;
      coord[1] = points[i].y;
      coord[2] = points[i].z;

      filePCD.write ( ( char* ) &(coord[0]), 3*sizeof ( float ) );
      //now, color

      unsigned char color[4];
      for(int j=0; j<3; j++)
        color[j] = colors[i].val[j];
      color[3] = 0;

      filePCD.write ( ( char* ) &(color[0]), 4*sizeof ( unsigned char ) );
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

    double  getReprjError ( const vector<vector<cv::Point3f> > &objectPoints,const vector< vector<cv::Point2f> >&points1,cv::Mat cam1,cv::Mat dist1,vector<cv::Mat>vT1,
                            const vector< vector<cv::Point2f> >&points2,cv::Mat cam2,cv::Mat dist2,vector<cv::Mat>vT2 ) {
        double  errReprj=0;
        int nPoints=0;
        for ( size_t i=0; i<points1.size(); i++ ) {

            //reproject points
            //move 3d points and project from 0

            cv::Mat  R,T;
            vector<cv::Point2f> pointsr1,pointsr2;
            getRTfromMatrix44 ( vT1[i],R,T );
            cv::projectPoints ( objectPoints[i] , R,T,cam1,dist1,pointsr1 );
            getRTfromMatrix44 ( vT2[i],R,T );
            cv::projectPoints ( objectPoints[i], R,T,cam2,dist2,pointsr2 );

            for ( size_t j=0; j<points1[i].size(); j++ ) {
                errReprj+= cv::norm ( points1[i][j]- pointsr1[j] );
                errReprj+= cv::norm ( points2[i][j]- pointsr2[j] );
                nPoints++;
            }

        }
        return errReprj/ ( 2*nPoints );
    }


    double  getReprjError ( const vector<cv::Point3f>  &objectPoints,const  vector<cv::Point2f> &points1,cv::Mat cam1,cv::Mat dist1,cv::Mat vT1,
                            const  vector<cv::Point2f> &points2,cv::Mat cam2,cv::Mat dist2,cv::Mat vT2 ) {
        vector<vector<cv::Point3f>   > _objectPoints;
        _objectPoints .push_back ( objectPoints );
        vector< vector<cv::Point2f> > _points1;
        _points1.push_back ( points1 );
        vector< vector<cv::Point2f> > _points2;
        _points2.push_back ( points2 );
        vector< cv::Mat >   _vT1;
        _vT1.push_back ( vT1 );
        vector< cv::Mat >   _vT2;
        _vT2.push_back ( vT2 );
        return getReprjError ( _objectPoints,_points1,cam1,dist1,_vT1, _points2,cam2,dist2,_vT2 );

    }

    double getReprjError ( const   vector<cv::Point3f>   &objectPoints,const   vector<cv::Point2f>  &points1,cv::Mat cam1,cv::Mat dist1, cv::Mat vT1 ) {
        vector<vector<cv::Point3f>   > _objectPoints;
        _objectPoints .push_back ( objectPoints );
        vector< vector<cv::Point2f> > _points1;
        _points1.push_back ( points1 );
        vector< cv::Mat >   _vT1;
        _vT1.push_back ( vT1 );
        return getReprjError ( _objectPoints,_points1,cam1,dist1,_vT1 );

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
        for ( int i=0; i<S.rows; i++ ) {
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
            return aruco_mm::getRTMatrix(r,t);
    }


    cv::Point3f  get3d ( cv::Point p, const cv::Mat & rangeMap, cv::Mat  cam, int wsize )  {
        if ( cam.type() !=CV_32F && cam.type() !=CV_64F )
            throw std::runtime_error ( "Invalid camera type" );

        if ( cam.rows!=3 || cam.cols!=3 )
            throw std::runtime_error ( "Invalid camera size" );

        cv::Point3f  xyz ( 0,0,0 );
        float cx,cy,fx_inv,fy_inv;
        if ( cam.type() ==CV_32F ) {
            cx=cam.at<float> ( 0,2 );
            cy=cam.at<float> ( 1,2 );
            fx_inv=1/cam.at<float> ( 0,0 );
            fy_inv=1./cam.at<float> ( 1,1 );
        } else if ( cam.type() ==CV_64F ) {
            cx=cam.at<double> ( 0,2 );
            cy=cam.at<double> ( 1,2 );
            fx_inv=1/cam.at<double> ( 0,0 );
            fy_inv=1./cam.at<double> ( 1,1 );
        }
        if ( wsize==0 ) {
            uint16_t d=rangeMap.at<uint16_t> ( p ) ;
            if ( d!=0 ) {
                xyz.z =  float ( d ) /1000.f ; // load and convert: mm -> meters
                xyz.x = xyz.z * ( p.x - cx ) * fx_inv;
                xyz.y= xyz.z * ( p.y - cy ) * fy_inv;
            } ;
        } else {
            //get average in window region
            //get limits
            cv::Point minmaxX=cv::Point ( std::max ( 0,p.x-wsize ),std::min ( p.x+wsize,rangeMap.cols-1 ) );
            cv::Point minmaxY=cv::Point ( std::max ( 0,p.y-wsize ),std::min ( p.y+wsize,rangeMap.rows-1 ) );
             //get median z and then compute
            vector<float> zs;
            for ( int y=minmaxY.x; y<=minmaxY.y; y++ ) {
                const uint16_t *d=rangeMap.ptr<uint16_t> ( y ) ;
                for ( int x=minmaxX.x; x<=minmaxX.y; x++ ) {
                    if ( d[x]!=0 ) zs.push_back ( float ( d[x] ) /1000.f );

                }
            }
            if ( zs.size() >=3 ) {
                std::sort ( zs.begin(),zs.end() );
                xyz.z=zs[ zs.size() /2];
                xyz.x = xyz.z * ( p.x - cx ) * fx_inv;
                xyz.y= xyz.z * ( p.y - cy ) * fy_inv;
            }
        }
        return xyz;
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

    std::vector<cv::Point2f> projectPoints(const vector<cv::Point3f> &p3d,   se3 &pose, const aruco::CameraParameters &cp){
        vector<cv::Point2f> p2d;
        cv::projectPoints(p3d,pose.getRvec(),pose.getTvec(),cp.CameraMatrix,cp.Distorsion ,p2d);
        return p2d;
    }

}
