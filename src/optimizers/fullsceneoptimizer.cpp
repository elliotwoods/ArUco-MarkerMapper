#include "fullsceneoptimizer.h"
#include "utils/utils3d.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>

#include <unordered_map>
namespace aruco_mm{
using namespace std;
double FullSceneOptimizer::optimize(
        MarkerSet & MarkerSet,
        FrameSet & FrameSet,const std::set<uint32_t> &fixedMarkers,aruco::CameraParameters &camp,bool fix_cameraparams){

    Params params;
    params.fixedMarkers=fixedMarkers;
    params.fix_camera_params=fix_cameraparams;
    auto res=optimize(MarkerSet,FrameSet,camp,params);
    return res;
}

SparseLevMarq<double>::eVector FullSceneOptimizer::toVector(MarkerSet & MarkerSet, FrameSet & FrameSet,  aruco::CameraParameters &cam_params){

    SparseLevMarq<double>::eVector sol;
    //create the solution vector first
    sol.resize(MarkerSet.size()*6+ FrameSet.size()*6  + (4+5));//markers +frames+camera parameters
    int offset=0;
    cv::Mat r,t;
    for(auto m:MarkerSet){
        aruco_mm::getRTfromMatrix44(m.second.rt_g2m,r,t);
        addRT(sol,offset,r,t);
        offset+=6;
    }
    for(auto f:FrameSet){
        aruco_mm::getRTfromMatrix44(f.second.rt_c2g,r,t);
        addRT(sol,offset,r,t);
        offset+=6;
    }
    //now, the camera params
    sol(offset++)=cam_params.CameraMatrix.at<float>(0,0);//fx
    sol(offset++)=cam_params.CameraMatrix.at<float>(1,1);//fy
    sol(offset++)=cam_params.CameraMatrix.at<float>(0,2);//cx
    sol(offset++)=cam_params.CameraMatrix.at<float>(1,2);//cy
    for(int i=0;i<5;i++)    sol(offset++)=cam_params.Distorsion.ptr<float>(0)[i];
    return sol;
}

void FullSceneOptimizer::fromVector(SparseLevMarq<double>::eVector  &sol,MarkerSet & MarkerSet, FrameSet & FrameSet,   aruco::CameraParameters &cam_params){
    cv::Mat r,t;

    //move data back to the original structures
    //determine the origin location and tranlate everyhing to ensure it is zero
    cv::Mat orgT=cv::Mat::eye(4,4,CV_32F);
    int offset=0;
    for(auto &m:MarkerSet){
        getRT(sol,offset,r,t);
        m.second.rt_g2m=aruco_mm::getRTMatrix(r,t);
        offset+=6;
        //        if (m.second.id==_originMarkerId) orgT=m.second.rt_g2m.inv();
    }
    //move everything to the origin marker
    for(auto &m:MarkerSet){
        m.second.rt_g2m=orgT*m.second.rt_g2m;
        //  cout<<m.second.rt_g2m<<endl;
    }
    for(auto &f:FrameSet){
        getRT(sol,offset,r,t);
        f.second.rt_c2g=orgT*aruco_mm::getRTMatrix(r,t);
        offset+=6;
    }
    //now, the camera params
    cam_params.CameraMatrix.at<float>(0,0)=sol(offset++);//fx
    cam_params.CameraMatrix.at<float>(1,1)=sol(offset++);//fy
    cam_params.CameraMatrix.at<float>(0,2)=sol(offset++);//cx
    cam_params.CameraMatrix.at<float>(1,2)=sol(offset++);//cy
    for(int i=0;i<5;i++)    cam_params.Distorsion.ptr<float>(0)[i]=sol(offset++);

}

std::vector<float> FullSceneOptimizer::getSolution(MarkerSet & MarkerSet, FrameSet & FrameSet,  aruco::CameraParameters &cam_params){
    SparseLevMarq<double>::eVector sol=toVector(MarkerSet,FrameSet,cam_params);
    vector<float> vsol(sol.size());
    for(int i=0;i<sol.size();i++) vsol[i]=sol(i);
    return vsol;
}

double FullSceneOptimizer::optimize(MarkerSet & MarkerSet, FrameSet & FrameSet,aruco::CameraParameters &cam_params, Params &params){
    sba_ba_params=params;
    sba_markerSet=MarkerSet;
    sba_frameSet=FrameSet;

    // sba_fixedmarkers=fixedMarkers;
    // sba_fix_cameraparams=fix_cameraparams;

    //create the solution vector first
    SparseLevMarq<double>::eVector sol=toVector(sba_markerSet,sba_frameSet,cam_params);

    cout<<"SOL SIZE="<<sol.size()<<" nf:"<<sba_frameSet.size()<<" nm:"<<sba_markerSet.size()<< endl;
    SparseLevMarq<double>::eVector error;

    SparseLevMarq<double>  solver;
     solver.verbose()=params.verbose;

    error_optimization(sol,error); cout<<"ERR INITIAL="<<error.cwiseProduct(error).sum()<<endl; stats(error);
   // cout<<"INI SOLUTION: ";for(int i=0;i<10;i++)cout<<sol(i)<<" ";cout<<endl;
    double minError=( float(error.size())/2.)*sba_ba_params.min_error;
    double minStepError=( float(error.size())/2.)*sba_ba_params.min_step_error;
// cout<<"MIN STEP ERROR="<<minStepError<<endl;

  //  solver.setStepCallBackFunc(std::bind(&FullSceneOptimizer::stepcallbackfunc,this,std::placeholders::_1));
    solver.setParams(sba_ba_params.max_iters,  minError,minStepError);
    solver.solve(sol,std::bind(&FullSceneOptimizer::error_optimization,this,std::placeholders::_1,std::placeholders::_2),std::bind(&FullSceneOptimizer::error_jacobian,this,std::placeholders::_1,std::placeholders::_2));
    error_optimization(sol,error); cout<<"ERRXX FINAL="<<error.cwiseProduct(error).sum()<<endl; stats(error);
   // cout<<"END SOLUTION: ";for(int i=0;i<10;i++)cout<<sol(i)<<" ";cout<<endl;

    //move data back to the original structures
    //determine the origin location and tranlate everyhing to ensure it is zero
    fromVector(sol,MarkerSet,FrameSet,cam_params);
    // cout<<"SOLUTION END: ";for(int i=0;i<sol.size();i++)cout<<sol(i)<<" ";cout<<endl;

    return error.cwiseProduct(error).sum();
}


void FullSceneOptimizer::stepcallbackfunc(const SparseLevMarq<double>::eVector  &sol) {
//    if(!outErrorFile.is_open()){
//        outErrorFile.open("pixerr.txt",std::ios::ate);
//        it_debug=0;
//    }
    SparseLevMarq<double>::eVector error;
    error_optimization(sol,error);
    double err=sqrt( error.cwiseProduct(error).sum()/(float(error.rows())/2.f));
   cout<<it_debug++<<" "<<err<<endl;

}


vector<cv::Point3f> FullSceneOptimizer::getMarkerPoints(float size,cv::Mat RT )
{
    float size_2=size/2.;
    //now, that the current location is estimated, add new markers and update old ones
    vector<cv::Point3f> points = { cv::Point3f ( -size_2, size_2,0 ),cv::Point3f ( size_2, size_2 ,0 ),
                                   cv::Point3f ( size_2, -size_2,0 ),cv::Point3f ( -size_2, -size_2,0 )  };
    if (!RT.empty())   aruco_mm::mult<cv::Point3f>(RT,points);

    return points;

}

inline double hubber(double e,double _delta){
double dsqr = _delta * _delta;
 if (e <= dsqr) { // inlier
   return  e;
 } else { // outlier
   double sqrte = sqrt(e); // absolut value of the error
   return  2*sqrte*_delta - dsqr; // rho(e)   = 2 * delta * e^(1/2) - delta^2
 }
}

inline double hubberMono(double e){
    if (e <= 5.991) { // inlier
      return  e;
    } else  // outlier
       return  4.895303872*sqrt(e) - 5.991; // rho(e)   = 2 * delta * e^(1/2) - delta^2
}

inline double getHubberMonoWeight(double SqErr,double Information){
     return sqrt(hubberMono(Information * SqErr)/ SqErr);
}
void FullSceneOptimizer::error_optimization(const SparseLevMarq<double>::eVector &sol,SparseLevMarq<double>::eVector &error)
{



    //a solution consists in analyzing the reprojection error according to the parameters
    //first extract camera
    cv::Mat camera=cv::Mat::eye(3,3,CV_32F),dist=cv::Mat(1,5,CV_32F);
    int cameraParamsOffset=sol.size() -9;
    int idx=cameraParamsOffset;
    camera.at<float>(0,0)=sol[ idx++];//fx,fy,cx,cy
    camera.at<float>(1,1)=sol[ idx++];//fx,fy,cx,cy
    camera.at<float>(0,2)=sol[ idx++];//fx,fy,cx,cy
    camera.at<float>(1,2)=sol[ idx++];//fx,fy,cx,cy
    for(int i=0;i<5;i++)   dist.at<float>(0,i)=sol[ idx++];
    //cout<<camera<<dist<<endl;
    //get 3d points of each marker according to this solution
    std::unordered_map<uint32_t,vector<cv::Point3f> > markers_points3d;
    {
        cv::Mat r,t;
        auto marker=sba_markerSet.begin();
        //extract r,t elements in matrices
        for(size_t i=0;i<sba_markerSet.size();i++,marker++){
            getRT(sol,i*6,r,t);
            //get the 3d points associated with this location
            markers_points3d.insert( std::make_pair(marker->second.id, getMarkerPoints(marker->second.markerSize,aruco_mm::getRTMatrix(r,t))));
        }
    }
    //compute size of error vector
    int errorvectorsize=0;
    for(auto frame: sba_frameSet) errorvectorsize+=frame.second.markers.size()*8;//each markers provides 4 points, with two measures each(x,y)
    error.resize(errorvectorsize);

    //compute the start erro row of each frame
    vector<int> frame_start;
    {
        int foffset=0;
        frame_start.push_back(foffset);
        for(auto frame:sba_frameSet ){
            foffset+=frame.second.markers.size()*8;
            frame_start.push_back(foffset);
        }
    }


    //obtain the projection in each marker frame
    //cout<<"Errorsize="<<errorvectorsize<<endl;
    vector<vector<Eigen::Triplet<double> > >jac_omp(omp_get_max_threads());

#pragma omp parallel for
    for(int curFrameIdx=0;curFrameIdx<int(sba_frameSet.size());curFrameIdx++){
        int omp_tid=omp_get_thread_num();
        FrameInfo frame=get(sba_frameSet,curFrameIdx);
        cv::Mat r,t;
        int frameOffset=6*sba_markerSet.size()+curFrameIdx*6;
        //extract r,t elements in matrices
        getRT(sol,frameOffset,r,t);
        //now, project the 3d points of the markers in the frame
        //gather them
        vector<cv::Point2f> m_points2d,m_points2d_observed;
        vector<cv::Point3f> m_points3d;
        for(auto m:frame.markers){//add the 3d points
            assert(markers_points3d.find(m.id)!=markers_points3d.end());
            m_points3d.insert(m_points3d.end(),markers_points3d[m.id].begin(),markers_points3d[m.id].end());
            m_points2d_observed.insert(m_points2d_observed.end(),m.begin(),m.end());
        }
        //project all these points
        cv::Mat jacobian;
        cv::projectPoints(m_points3d,r,t,camera,dist,m_points2d,jacobian);
        //        //add the 2dpoints to the output vector
        int currErrRow=frame_start[curFrameIdx];
        for(size_t  p=0;p<m_points2d.size();p++,currErrRow+=2) {
            double ex=m_points2d[p].x-m_points2d_observed[p].x;
            double ey=m_points2d[p].y-m_points2d_observed[p].y;

            float robust_weight= getHubberMonoWeight((ex*ex+ ey*ey),1);
//            error(currErrRow)= sba_ba_params.error_func(ex) ;
//            error(currErrRow+1)= sba_ba_params.error_func(ey);

            error(currErrRow)= robust_weight* ex ;
            error(currErrRow+1)= robust_weight* ey;

            if (sba_ba_params.fixedFrames.find(frame.frame_idx)==sba_ba_params.fixedFrames.end()){//not a fixed frame
                //r and t
                for(int rti=0;rti<6;rti++){
                    jac_omp[omp_tid].push_back(Eigen::Triplet<double>(currErrRow,frameOffset+rti,jacobian.at<double>(p*2,rti)));
                    jac_omp[omp_tid].push_back(Eigen::Triplet<double>(currErrRow+1,frameOffset+rti,jacobian.at<double>(p*2+1,rti)));
                }
            }
            if (!sba_ba_params.fix_camera_params ){
                //camera paramrs
                for(int rti=0;rti<9;rti++){
                    jac_omp[omp_tid].push_back(Eigen::Triplet<double>(currErrRow,cameraParamsOffset+rti,jacobian.at<double>(p*2,rti+6)));
                    jac_omp[omp_tid].push_back(Eigen::Triplet<double>(currErrRow+1,cameraParamsOffset+rti,jacobian.at<double>(p*2+1,rti+6)));
                }
            }
        }
    }
    _jacobian.clear();

    for(auto v:jac_omp)
        _jacobian.insert(_jacobian.end(),v.begin(),v.end());



}

cv::Mat FullSceneOptimizer::getRTMatrix ( SparseLevMarq<double>::eVector  & rt ){
    cv::Mat r(1,3,CV_32F),t (1,3,CV_32F);
    for(int i=0;i<3;i++){
        r.ptr<float>(0)[i]=rt(i);
        t.ptr<float>(0)[i]=rt(i+3);
    }
    return aruco_mm::getRTMatrix(r,t);
}

void FullSceneOptimizer::error_jacobian(const SparseLevMarq<double>::eVector  & sol,  Eigen::SparseMatrix<double> &sp){
    //only the jacobian of the markers remain to be computed
    //a solution consists in analyzing the reprojection error according to the parameters
    //first extract camera
    cv::Mat camera=cv::Mat::eye(3,3,CV_32F),dist=cv::Mat(1,5,CV_32F);
    int cameraParamsOffset=sol.size() -9;
    int idx=cameraParamsOffset;
    camera.at<float>(0,0)=sol[ idx++];//fx,fy,cx,cy
    camera.at<float>(1,1)=sol[ idx++];//fx,fy,cx,cy
    camera.at<float>(0,2)=sol[ idx++];//fx,fy,cx,cy
    camera.at<float>(1,2)=sol[ idx++];//fx,fy,cx,cy
    for(int i=0;i<5;i++)   dist.at<float>(0,i)=sol[ idx++];

    //where a frame projects to
    struct marker_frame_projection{
        marker_frame_projection(int f,int s,cv::Mat R,cv::Mat T){frame_idx=f;start_row=s;frame_r=R;frame_t=T;}
        int frame_idx;//frame in which projects
        int start_row;//row where the error starts
        cv::Mat frame_r,frame_t;//frame r and t according to this solution
    };
    //for each marker
    vector<vector<Eigen::Triplet<double> > >jac_omp(omp_get_max_threads());
#pragma omp parallel for
    for(int markerIdx=0;markerIdx<int(sba_markerSet.size());markerIdx++){
        auto marker=get(sba_markerSet,markerIdx);
        if (sba_ba_params.fixedMarkers.find( marker.id )==sba_ba_params.fixedMarkers.end() ){//The fixd markers derivatives are set of 0 der to avoid moving them
            //where are his projections?
            int proj_off=0;//counter
            std::vector <marker_frame_projection> mfv;//where it projects
            int fidx=0;
            for(auto frame:sba_frameSet){//check across frames
                for(auto m:frame.second.markers){//see if the marker is in the frame
                    if (m.id==marker.id) {
                        cv::Mat r,t;
                        getRT(sol,sba_markerSet.size()*6+fidx*6,r,t);
                        mfv.push_back(marker_frame_projection(frame.second.frame_idx,proj_off,r,t));//add it to list
                    }
                    proj_off+=8;//each markers are 8 errors rows
                }
                fidx++;
            }


            //for each component of the position (rx,ry...tz)
            double der_epsilon=1e-3;

            for(int rt=0;rt<6;rt++){
                //get the altered vector adding and substracting epison
                SparseLevMarq<double>::eVector rt_p=sol.middleRows(markerIdx*6,6);
                rt_p(rt)+=der_epsilon;
                SparseLevMarq<double>::eVector rt_n=sol.middleRows(markerIdx*6,6);
                rt_n(rt)-=der_epsilon;
                //the marker points joined in one vector
                auto obj_points_p_n=getMarkerPoints(marker.markerSize,getRTMatrix(rt_p));
                auto obj_points_n=getMarkerPoints(marker.markerSize,getRTMatrix(rt_n));
                obj_points_p_n.insert(obj_points_p_n.end(),obj_points_n.begin(),obj_points_n.end());
                //project them in each and every frame where the marker is visible
                for(auto mfp: mfv ){
                    //get the frame location and project
                    vector<cv::Point2f> img_points_p_n;
                    cv::projectPoints(obj_points_p_n,mfp.frame_r,mfp.frame_t,camera,dist,img_points_p_n);

                    //makes the difference to compute the derivate
                    int errorRow= mfp.start_row;
                    int omp_thread=omp_get_thread_num();
                    for(int p=0;p<4;p++){
                        double derv_errx=(img_points_p_n[p].x-img_points_p_n[p+4].x)/(2.*der_epsilon);
                        jac_omp[omp_thread].push_back(Eigen::Triplet<double>( errorRow++,markerIdx*6+rt,derv_errx));
                        double derv_erry=(img_points_p_n[p].y-img_points_p_n[p+4].y)/(2.*der_epsilon);
                        jac_omp[omp_thread].push_back(Eigen::Triplet<double>( errorRow++,markerIdx*6+rt,derv_erry));

                    }
                }
            }
        }

    }
    //for (auto t:_jacobian) cout<<"("<<t.col()<<" "<<t.row()<<" )";
    //determine each marker
    //build the sparse matrix
    for(auto v:jac_omp)
        _jacobian.insert(_jacobian.end(),v.begin(),v.end());
    sp.setFromTriplets(_jacobian.begin(),_jacobian.end());



}


void FullSceneOptimizer::addRT(  SparseLevMarq<double>::eVector &sol,int start,const cv::Mat &r,const cv::Mat &t){
    assert(r.type()==t.type());
    if (r.type()==CV_32F){
        for(int j=0;j<3;j++){
            sol[start+j]=r.ptr<float>(0)[j];
            sol[start+j+3]=t.ptr<float>(0)[j];
        }
    }else{
        for(int j=0;j<3;j++){
            sol[start+j]=r.ptr<double>(0)[j];
            sol[start+j+3]=t.ptr<double>(0)[j];
        }

    }

}
void FullSceneOptimizer::getRT(const SparseLevMarq<double>::eVector &sol,int start,cv::Mat &r,cv::Mat &t){
    r.create(1,3,CV_32F);t.create(1,3,CV_32F);
    for(int j=0;j<3;j++){
        r.ptr<float>(0)[j]=sol[start+j];
        t.ptr<float>(0)[j]=sol[start+j+3];
    }
}

void FullSceneOptimizer::stats( SparseLevMarq<double>::eVector  &error){
    double maxErr=std::numeric_limits<double>::lowest(),minErr=std::numeric_limits<double>::max();
    double avr_err=0;
    for(int i=0;i<error.size();i++)avr_err+=fabs(error(i));
    avr_err/=double(error.size());
    cout<<"avg error="<<avr_err;
    for(int i=0;i<error.size();i++) {
        if (maxErr< fabs(error(i))) maxErr=fabs(error(i));
        if (minErr> fabs(error(i))) minErr=fabs(error(i));
    }
    cout<<"  min Max ="<<minErr<<" "<<maxErr<<endl;

}

}
