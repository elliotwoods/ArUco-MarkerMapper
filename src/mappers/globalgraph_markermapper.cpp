#include "globalgraph_markermapper.h"
#include "debug.h"
#include "optimizers/fullsceneoptimizer.h"
#include "optimizers/sparselevmarq.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "optimizers/ippe.h"
#include "posegraphoptimizer.h"
#include <list>
namespace aruco_mm{
using namespace std;

bool GlobalGraphMarkerMapper::process (const arucoMarkerSet &in_detected_markers,int imageIdx ,bool forceConnectedComponent ) {
    detected_markers=in_detected_markers;
    if (imageIdx!=-1) _frameCounter=imageIdx;
    else _frameCounter++;
    if (detected_markers.size()<2){
        markermapper_debug_msg("Not added because need at least two markers",5);
        return false;
    }

    //check if any of the markers is already in the map, otherwise, we will not consider the addition
    if ( forceConnectedComponent && kdtrees.size()!=0){
        bool foundAny=false;
        for(auto marker:detected_markers){
            if (kdtrees.count(marker.id)!=0){
                foundAny=true;
                break;
            }
        }
        if (!foundAny){
            markermapper_debug_msg("Not added because it is not connected to an already added marker. No connection with the group",5);
            return false;

        }
    }






    float cos_limit=-cos( max_marker_cam_angle *3.141516/180.);
    accepted_markers.clear();
    bool addFrame=false;
    MarkerSet ms;
    for(const auto &marker:detected_markers){
        //compute 3d pose to each marker
        cv::Mat mpose=solvePnP(getMarkerPoints(_defaultMarkerSize),marker,_cam_params);
        se3 pose=mpose;
        ms[marker.id]=MarkerInfo(marker.id,mpose,_defaultMarkerSize);
        //filter out by camera-marker angle. If too high, discard
        bool angle_too_high=false;
        {
            //cerr<<"angle     "<<marker.id<<" "<<acos( normal_.z)*180./M_PI <<" "<<endl;
            if (cameraMarkerDotProduct(mpose)>cos_limit){
              angle_too_high=true; //45deg
            }
        }

        if (!angle_too_high){
            accepted_markers.insert(marker.id);
            bool add2kdtree=false;
            //if it is far from its other locations, add the position and the frame
            //if it is first time the marker is seen, add
            if (kdtrees.find(marker.id)==kdtrees.end()) add2kdtree=true;
            else{//is the pose far from the rest of locations?
                auto & kdtree=kdtrees[marker.id];//search for nearby locations
                std::vector<std::pair<size_t,float> > poses=kdtree.radiusSearch(pose,radius_search_t);
                if (poses.size()==0)add2kdtree=true;//no one in the radious
                else{//check the angular difference analyzing the dot product
                    //find minimum
                    float minDot=std::numeric_limits<float>::max();
                    for(auto p:poses) minDot=std::min(float(minDot),fabs(pose.r_dot(kdtree.s3s[p.first])));
                    if(minDot<max_angular_dot_value) add2kdtree=true;//add if above a threshold

                }
            }
            if(add2kdtree ){//add point to its kdtree
                addFrame=true;
                kdtrees[marker.id].add_and_build(pose);
            }
        }
    }
    //add the frame

    if (addFrame)    {
        markermapper_debug_msg("Adding frame "<<_frameCounter,5);
        _frameSet[_frameCounter]=FrameInfo(_frameCounter,detected_markers);;
        for(auto d:detected_markers)
            markermapper_debug_msg("    marker "<<d.id,5);

        return true;
    }
    else {
        markermapper_debug_msg("No new relevant information. Move the camera",5);
        return false;
    }


}




void GlobalGraphMarkerMapper::optimize_impl (   ) {
       auto _to_string=[](int i){ 	std::stringstream str;str<<i;return str.str(); };
  //debug::Debug::setLevel(10);
 // finalize_();


    //reset information
    _markerSet.clear();

    //obtains the initial pose graph
    std::map<uint32_t,uint32_t>   markers_id_pos;//given an id, returns its position in the poseGraph
    StgMatrix<se3> poseGraph;//pairwise poses
    graph::Graph<int> cost_graph;//cost between nodes in terms of reprojection error
    std::map<uint32_t,uint32_t>   markers_pos_id;//given a position in the posegraph, returns id

    bool read_from_debug_file=false;
    //if in full debug mode, try reading from a file
    if (debug::Debug::getLevel()>=11)
        read_from_debug_file=debug_read("debug_global_graph.bin",poseGraph,cost_graph,markers_id_pos,markers_pos_id);


     //not read from file, do the real processing
    if (!read_from_debug_file){
        compute_pairwise_information(_frameSet,_cam_params,poseGraph,cost_graph,markers_id_pos);
        for(auto mip:markers_id_pos) markers_pos_id.insert(make_pair(mip.second,mip.first));
    }

    //-------------------------------------------------------------
    auto time2=std::chrono::high_resolution_clock::now();

    StgMatrix<aruco_mm::Pose_Error> PoseGraph;
    uint32_t nnodes=poseGraph.cols();
    assert(poseGraph.cols()==poseGraph.rows());
    PoseGraph.resize(nnodes,nnodes);
    vector<std::string> nodeInfo;
    for(size_t i=0;i<nnodes;i++)
        nodeInfo.push_back(_to_string(markers_pos_id[i]));
    for(size_t i=0;i<nnodes;i++)
        for(size_t j=0;j<nnodes;j++){
            PoseGraph(i,j).pose=poseGraph(i,j);
            PoseGraph(i,j).repj_error=cost_graph(i,j);
         }
    aruco_mm::PoseGraphOptimizer PGO;
    uint32_t best_org=PGO.optimize(PoseGraph,true,nodeInfo);
    poseGraph.resize(nnodes,nnodes);
    cost_graph.setParams(nnodes,false);
    for(size_t i=0;i<nnodes;i++)
        for(size_t j=0;j<nnodes;j++){
            poseGraph(i,j)=PoseGraph(i,j).pose;
            cost_graph(i,j)=PoseGraph(i,j).repj_error;
            cost_graph(i)=markers_pos_id.at(i);
        }


    //-------------------------------------------------------------

    //let us propagate and see the results
    uint32_t origin_id=markers_pos_id[best_org];

    graph::Floyd falgo;
    falgo.process(cost_graph);
    _markerSet=calculateMarkerLocationsFromExpansionGraph(falgo,poseGraph,origin_id,markers_id_pos,markers_pos_id);

    if (debug::Debug::getLevel()>=10) savePCDFile("ss2.pcd",_markerSet);

    //if origin id is not the one requested, then do the appropriate transform
    if (_originMarkerId!=-1 && _originMarkerId!=int(origin_id)){
        //ok, get the rigid transform to move all markers
        auto p_a=_markerSet[_originMarkerId].get3dPoints();
        auto p_b=_markerSet[origin_id].get3dPoints();
        auto rt=rigidBodyTransformation_Horn1987(p_a,p_b);
        //apply to all
        for(auto &m:_markerSet)
            m.second.rt_g2m= rt*m.second.rt_g2m;

    }
    else _originMarkerId=origin_id;

    if (debug::Debug::getLevel()>=10) savePCDFile( "outxx.pcd",_markerSet);

    auto time3=std::chrono::high_resolution_clock::now();
    cout<<"PosegraphOptimization="<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(time3-time2).count())/1e9<<"s"<<endl;


    //Full Optimization,
    markermapper_debug_msg("Full optimization",5);
    aruco_mm::FullSceneOptimizer::Params params;
    aruco_mm::FullSceneOptimizer opt;


    params.fixedMarkers={uint32_t(_originMarkerId)};
    params.max_iters=100;
    params.min_error=0.5;
    params.min_step_error=0.05;
    params.verbose=true;
    params.fix_camera_params=!_optimizeCameraIntrinsics;


    //Do a full refinament. Go adding frames if their errors are below a threshold. Then, repeat until no more frames added
    FrameSet  fs_opt =getOptimizingFrameSet(_frameSet, _markerSet,_cam_params,1e6,2);
    cout<<"fs_opt ="<<fs_opt .size()<<endl;

    int cur_size=fs_opt.size();
    int prev_size=0;
    while(cur_size!=prev_size)//while elements added, keep optimizing
    {
        opt.optimize( _markerSet,fs_opt,_cam_params,params);
        //now, take these not added and add them now to optimization
        FrameSet unused_fs;
        for(auto frame:_frameSet)
            if (  fs_opt.find(frame.first)==fs_opt.end()) unused_fs.insert(frame);

        unused_fs=getOptimizingFrameSet(unused_fs, _markerSet,_cam_params,1e6,2);
        //add the elements returned to the optimized one , and reoptimize
        for(auto frame:unused_fs) fs_opt.insert(frame);
        swap(cur_size,prev_size);
        cur_size=fs_opt.size();

    };

    if (debug::Debug::getLevel()>=10) savePCDFile( "outxx2.pcd",_markerSet);
    //save to the class variable
    _frameSet=fs_opt;
    for(auto me:getAvrgMarkerRepjError(_markerSet,_frameSet,_cam_params)){
        markermapper_debug_msg( "(Marker "<<me.first<<" reprjError="<<me.second<<")",0);
    }
    auto time4=std::chrono::high_resolution_clock::now();

    cout<<"BundleAdjustmentOptimization="<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(time4-time3).count())/1e9<<"s"<<endl;



}
//one of the keys of this method. Compute a posegraph.
//first, compute for each frame, a tentative relative position between every two markers
//then, we select as best interframe pose as the one that minimizes the reprojection error in all the frames where the two frames are seen
void GlobalGraphMarkerMapper::compute_pairwise_information(FrameSet &fs,const aruco::CameraParameters &cam_params,StgMatrix<se3> &pose_graph,graph::Graph<int> &cost_graph,std::map<uint32_t,uint32_t>   & markers_id_pos ){
    //first, determine the total number of different markes
    markers_id_pos.clear();//id-position in the graph
    std::set<uint32_t> marker_ids;

    for(auto f:fs)    for(auto m:f.second.markers) marker_ids.insert(m.id);

    int pos=0;
    for(auto m:marker_ids) markers_id_pos.insert( std::make_pair(m,pos++) );

    std::map<uint32_t,uint32_t>  markers_pos_id;//id-position in the graph
    for(auto mip:markers_id_pos) markers_pos_id.insert(make_pair( mip.second,mip.first));

    //now, create the "graph" with relations
    struct pose_frame{pose_frame(const cv::Mat &p,uint32_t f):pose(p),frame(f){}  cv::Mat pose;uint32_t frame;};

    StgMatrix<  std::vector<pose_frame> > graph (markers_id_pos.size(),markers_id_pos.size());

    std::map<uint32_t,std::map<uint32_t,cv::Mat> > marker_frame_pose;
    for(auto m:markers_id_pos) marker_frame_pose[m.first]=std::map<uint32_t,cv::Mat> ();

    //for each frame, take marker pair and compute the relation between them
#pragma omp parallel for
    for(int fidx=0;fidx<int(fs.size());fidx++){
        auto frame=at(fs,fidx);
      //  cerr<<frame.frame_idx<<" ";
        //determine the inter markers relations and store them
        std::vector<cv::Mat> marker_poses( frame.markers .size());
        //first , determine the marker to camera pose
        for(size_t m=0;m< frame.markers.size();m++ ){
            auto marker=frame.markers[m];
            auto poses=IPPE::solvePnP_(getMarkerPoints(_defaultMarkerSize),marker,cam_params.CameraMatrix,cam_params.Distorsion);
            if (poses[0].second/poses[1].second<0.6) {
                marker_poses[m]=poses[0].first; ;
#pragma omp critical
                    marker_frame_pose[marker.id][frame.frame_idx]=poses[0].first;
                }
            }

        for( int i=0;i<int(frame.markers.size())-1;i++)
            for( int j=i+1;j<int(frame.markers.size());j++){
                //graph indices
                uint32_t mid_i_idx= markers_id_pos[ frame.markers[i].id ];
                uint32_t mid_j_idx= markers_id_pos[ frame.markers[j].id ];
                if (!marker_poses[j].empty() && !marker_poses[i].empty()){
                    cv::Mat j2i=marker_poses[j].inv()*marker_poses[i];

#pragma omp critical
                    {
                        graph (mid_i_idx , mid_j_idx ).push_back(pose_frame(  j2i,frame.frame_idx));
                        graph( mid_j_idx , mid_i_idx ).push_back( pose_frame( j2i.inv(),frame.frame_idx));

                    }
                }
            }
    }

    //remove elements with less than n connections with the rest of elements


 auto undistort=[](const vector<cv::Point2f> &in,vector<cv::Point2f> &undp_points,const cv::Mat & CameraMatrix,const cv::Mat  &Distorsion){
  float cx=CameraMatrix.at<float>(0,2);
  float cy=CameraMatrix.at<float>(1,2);
  float fx=CameraMatrix.at<float>(0,0);
  float fy=CameraMatrix.at<float>(1,1);
  cv::undistortPoints(in,undp_points,CameraMatrix,Distorsion);
  //now, move points back to pixel values
  for(auto &p:undp_points){
      p.x= p.x*fx + cx;
      p.y= p.y*fy + cy;
  }
 };

//fast version of the reproj error. Is not really the reprj error, but is proportional. Input 2d points are already undistorted
    auto get_rpj_err=[](const vector<cv::Point3f> &p3d,const vector<cv::Point2f> &pd_und, const cv::Mat &T,const cv::Mat & CameraMatrix ){
        float cx=CameraMatrix.at<float>(0,2);
        float cy=CameraMatrix.at<float>(1,2);
        float fx=CameraMatrix.at<float>(0,0);
        float fy=CameraMatrix.at<float>(1,1);
       const float *t=T.ptr<float>(0);
        //now, project
        double err=0;
        for(size_t i=0;i<p3d.size();i++){
            //transform 3d point
            cv::Point3f p;
            p.x= p3d[i].x*t[0]+ p3d[i].y*t[1]+ p3d[i].z*t[2]+ t[3];
            p.y= p3d[i].x*t[4]+ p3d[i].y*t[5]+ p3d[i].z*t[6]+ t[7];
            p.z= p3d[i].x*t[8]+ p3d[i].y*t[9]+ p3d[i].z*t[10]+ t[11];
            //now, project
            cv::Point2f pp;
            pp.x= (p.x/p.z)*fx+cx;
            pp.y= (p.y/p.z)*fy+cy;
//            err+=cv::norm (pp-pd_und[i]);
            err+= sqrt((pp.x- pd_und[i].x) * (pp.x- pd_und[i].x)    +  (pp.y- pd_und[i].y) * (pp.y- pd_und[i].y));
        }
        return err*0.25;
    };

    cerr<<"\n-------------------"<<endl;
    //Now,from the set of possible poses between every two frames, select the one that minimizes the reprojection error in all views of the two markers
    //the result is both a posegraph, and a cost graph. The first has the pose and the second the avrg reprojection error. So, poses with low cost are better options

    StgMatrix<se3> aux_pose_graph;
    aux_pose_graph.resize(markers_id_pos.size(),markers_id_pos.size());
    graph::Graph<int>  aux_cost_graph;
    aux_cost_graph.setParams(markers_id_pos.size(),false);
    aux_cost_graph.setDiagonalLinkValues(0);
    auto marker_points=getMarkerPoints(_defaultMarkerSize);
#pragma omp parallel for
    for( int i=0;i< int(markers_id_pos.size());i++){
         auto mi_id=aux_cost_graph(i)=markers_pos_id[i];//do not forget setting the id in the cost_graph!!!!! It will be used later
         std::map<uint32_t,vector<cv::Point2f> > frame_undpoints;//store the undistorted version of the points of mi_id marker in the frame to avoid recalculation
        // cerr<<i<< " ";
        for( size_t j=i+1;j< markers_id_pos.size();j++){
            if ( graph(i,j).size()>=size_t(min_num_edges_between_nodes)){//require at least min_num_edges_between_nodes views to consider the link
                //for each possible pose, compute the reprjection error to the rest adn get the one that minimizes
                se3 best_pose;
                double minErr=std::numeric_limits<double>::max();
                for(auto pose_frame:graph(i,j)){
                    double err=0;
                    for(auto pf_t:graph(i,j)){
                        auto mj_id=markers_pos_id[ j];
                        //compute matrix transforming marker i to camera according to marker j
                        auto c2m=  marker_frame_pose[ mj_id][pf_t.frame]* pose_frame.pose  ;
                        if (frame_undpoints[pf_t.frame].size()==0) //if the 2d points are not yet undistorted, do it now
                            undistort(fs[pf_t.frame].markers.get(mi_id),frame_undpoints[pf_t.frame],cam_params.CameraMatrix,cam_params.Distorsion);
                        //now, compute repj error
//                        cout<<getReprjError(marker_points,fs[pf_t.frame].markers.get(mi_id),cam_params.CameraMatrix,cam_params.Distorsion,c2m)<<" "<<get_rpj_err(marker_points,frame_undpoints[pf_t.frame],c2m,cam_params.CameraMatrix)<<endl;
//                        cin.ignore();
                       err+=get_rpj_err(marker_points,frame_undpoints[pf_t.frame],c2m,cam_params.CameraMatrix);
//                        err+=getReprjError(marker_points,fs[pf_t.frame].markers.get(mi_id),cam_params.CameraMatrix,cam_params.Distorsion,c2m);
                    }
                    //   cout<<"conn "<<i<<" "<<j<<"  "<<err/double(graph[i][j].size())<<endl;
                    if (err <minErr){
                        minErr=err;
                        best_pose=pose_frame.pose;
                    }
                }
                markermapper_debug_msg("Best conn "<<i<<" "<<j<<"  "<<minErr/double(graph(i,j).size()),8);
                aux_pose_graph(i,j)= best_pose;
                aux_pose_graph(j,i)=best_pose.inverse();
                aux_cost_graph(i,j)=aux_cost_graph(j,i)= minErr/double(graph(i,j).size());
            }
        }
    }


    //now, let us clean up markers that are not connected.
vector<uint32_t> nodes_tokeep;

    vector<uint32_t> to_remove;
    vector<bool> bremove(aux_cost_graph.size(),false);
    std::map<uint32_t,uint32_t> old_new_indices;
     for(int i=0;i<aux_cost_graph.size();i++){
        bool has_connection=false;
        for(int j=0;j<aux_cost_graph.size() && !has_connection;j++)
            if (aux_cost_graph.areConnected(i,j)&& i!=j) has_connection=true;
        if (!has_connection) {bremove[i]=true;to_remove.push_back(i);}
        else {nodes_tokeep.push_back(i);}
    }

    auto gg=extractNodes(aux_cost_graph,aux_pose_graph,nodes_tokeep);
     cost_graph=gg.first;
     pose_graph=gg.second;
    //recreate the marker_id_pos
    markers_id_pos.clear();
    for(int i=0;i<cost_graph.size();i++)
        markers_id_pos[ cost_graph(i)]=i;



}



FrameSet GlobalGraphMarkerMapper::getOptimizingFrameSet(FrameSet &fset, MarkerSet &mset, aruco::CameraParameters cam_params, double maxRerpjErr, int minNMarkers){
    FrameSet  c_frameSet;
    markermapper_debug_msg("GetOptimizingFrameSet...",5);
     //get the set of ids
    std::set<uint32_t> ids_set;
    for(auto m:mset) ids_set.insert(m.first);
    vector< FrameSet> fset_omp(omp_get_max_threads());
    int ninvalidframes=0;

    auto marker_points=getMarkerPoints(_defaultMarkerSize);
    #pragma omp parallel for
    for(int i=0;i<int(fset.size());i++){
        auto frame=at(fset,i);
        auto used_markers=deleteAllBut(frame.markers,ids_set);
        if (int(used_markers.size())>=minNMarkers){
            //get the estimation of each marker first
            vector<cv::Mat> allposes;
            vector<cv::Point3f> objpoints;
            vector<cv::Point2f> imgpoints;
            for(auto marker:used_markers){
                assert(mset.find(marker.id)!=mset.end());
                auto p3d=mset[marker.id].get3dPoints();//3d points of the marker
                objpoints.insert(objpoints.end(),p3d.begin(),p3d.end());//add to vector to compute reprojections later
                imgpoints.insert(imgpoints.end(),marker.begin(),marker.end());//add to vector
                //get pose c2m
                auto poses=IPPE::solvePnP(marker_points,marker,cam_params.CameraMatrix,cam_params.Distorsion);

                //move the pose to be c2g
                for(auto pose:poses){
                    assert(mset.find(marker.id)!=mset.end());
                    allposes.push_back(  pose *mset[marker.id].rt_g2m.inv());
            }
            }
            //now, select the one that minimizes the reprojection error in all markers
            pair<int,double> min_repj(-1,std::numeric_limits<double>::max());
            for(size_t idx=0;idx<allposes.size();idx++){
                cv::Mat rvec,tvec;
                getRTfromMatrix44(allposes[idx],rvec,tvec);
                vector<cv::Point2f> reprj;
                cv::projectPoints(objpoints,rvec,tvec,cam_params.CameraMatrix,cam_params.Distorsion,reprj);
                double err=0;
                for(size_t p=0;p<reprj.size();p++)
                    err+=cv::norm(reprj[p] -imgpoints[p]);
                err/=double(reprj.size());

                if (err<min_repj.second && err<maxRerpjErr) min_repj=std::make_pair(idx,err);
                // cout<<rvec<<" "<<tvec<<":"<<err<<endl;
            }
            if ( min_repj.first!=-1){
                frame.rt_c2g= allposes[min_repj.first]; //solvePnP(used_markers, mset,cam_params,true);
                frame.markers=used_markers;
                fset_omp[omp_get_thread_num()].insert( make_pair(frame.frame_idx,frame) );
            }
            else{ninvalidframes++;}
        }
    }
    markermapper_debug_msg("NofInvalid frames="<<ninvalidframes,5);
    //now, join
    for(auto fs: fset_omp)
        for(auto frame:fs)
            c_frameSet.insert(frame);

    return c_frameSet;
}






MarkerSet GlobalGraphMarkerMapper::calculateMarkerLocationsFromExpansionGraph(graph::Floyd & falgo,  StgMatrix<se3> &poseGraph, uint32_t origin_id,std::map<uint32_t,uint32_t> &markers_id_pos,std::map<uint32_t,uint32_t> &markers_pos_id)
{
    (void)markers_pos_id;
    int largest_path=-1;
    MarkerSet mset;
    mset[origin_id]=MarkerInfo(origin_id,cv::Mat::eye(4,4,CV_32F),_defaultMarkerSize);
    for(auto m:markers_id_pos){
        if (m.first!=origin_id){
            cv::Mat RT=cv::Mat::eye(4,4,CV_32F);
            auto path=falgo.path(markers_id_pos[origin_id],markers_id_pos[m.first]);
            if (int(path.size())>largest_path)largest_path=int(path.size());

            for(size_t p=1;p<path.size();p++){
                assert(poseGraph(path[p-1] , path[p]).isValid());
                RT=   poseGraph(path[p-1] , path[p]).convert() *RT;
            }
            mset[ m.first]=MarkerInfo(m.first,RT.inv(), _defaultMarkerSize);
        }
    }
    cout<<"largest_path="<<largest_path<<endl;
    return mset;
}
void  GlobalGraphMarkerMapper::fromStream ( istream &str )  {
(void)str;
}

void  GlobalGraphMarkerMapper::toStream ( ostream &str )  {
    (void)str;

}

void GlobalGraphMarkerMapper::drawDetectedMarkers ( cv::Mat &imshow,int border_width) {

    for ( auto &m:detected_markers )
        if(accepted_markers.find(m.id)!=accepted_markers.end())
            m.draw ( imshow,cv::Scalar ( 0,0,255 ) ,border_width);
        else
            m.draw ( imshow,cv::Scalar ( 100,100,100 ) ,border_width);
}

void GlobalGraphMarkerMapper::map_toStream(std::map<uint32_t,uint32_t> &t,ostream &str ){
    uint32_t s=t.size();
    str.write ( ( char* ) &s ,sizeof ( uint32_t ) );
    for ( auto &x: t) {
        str.write ( ( char* ) &x.first,sizeof ( x.first ) );
        str.write ( ( char* ) &x.second,sizeof ( x.second) );
    }
}
void GlobalGraphMarkerMapper::map_fromStream(std::map<uint32_t,uint32_t> &t,istream &str ){
    //now, the map
    uint32_t s;
    str.read ( ( char* ) &s,sizeof ( s ) );
    t.clear();
    for ( uint32_t i=0; i<s; i++ ) {
        uint32_t key,val;
        str.read ( ( char* ) &key,sizeof ( key ) );
        str.read ( ( char* ) &val,sizeof ( val ) );
        t.insert ( make_pair ( key,val ) );
    }
}

void GlobalGraphMarkerMapper::debug_save(string fp,StgMatrix<se3> &poseGraph, graph::Graph<int> &cost_graph,std::map<uint32_t,uint32_t>   &markers_id_pos,std::map<uint32_t,uint32_t>   &markers_pos_id){

    ofstream file(fp);
    if(!file)throw std::runtime_error("Could not open file") ;
    poseGraph.toStream(file);
    cost_graph.toStream(file);
    map_toStream(markers_id_pos,file);
    map_toStream(markers_pos_id,file);

}

bool GlobalGraphMarkerMapper::debug_read(string fp,StgMatrix<se3> &poseGraph, graph::Graph<int> &cost_graph,std::map<uint32_t,uint32_t>   &markers_id_pos,std::map<uint32_t,uint32_t>   &markers_pos_id){
    ifstream file(fp);
    if (!file)return false;
    poseGraph.fromStream(file);
    cost_graph.fromStream(file);
    map_fromStream(markers_id_pos,file);
    map_fromStream(markers_pos_id,file);
    return true;

}


//extracts the nodes from a graph and creates new graphs
std::pair< graph::Graph<int>,StgMatrix<se3> >   GlobalGraphMarkerMapper::extractNodes(graph::Graph<int> &cost_graph,StgMatrix<se3> &poseGraph,const vector<uint32_t>  &nodes ){


    std::map<uint32_t,uint32_t> old_new_indices;
    int nidx=0;
    for(auto n:nodes) old_new_indices.insert(make_pair(n,nidx++));
    std::pair< graph::Graph<int>,StgMatrix<se3> > res;
    res.first.setParams(nodes.size(),false);
    res.second.resize(nodes.size(),nodes.size());
    for(auto ii:old_new_indices){
        res.first(ii.second)=cost_graph(ii.first);
        for(auto ii2:old_new_indices){
            res.first(ii.second,ii2.second) = cost_graph(ii.first,ii2.first);
            res.second(ii.second,ii2.second)= poseGraph(ii.first,ii2.first);
        }
    }
    return res;
}

}
