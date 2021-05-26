#ifndef _GlobalGraph_MarkerMapper_H
#define _GlobalGraph_MarkerMapper_H
#include "markermapper.h"
#include "marker_mapper_exports.h"
#include <unordered_set>
#include <cstdint>
#include "utils/kdtree.h"
#include "stgmatrix.h"

#include "graph.h"
#include <mutex>
namespace aruco_mm{
class  MARKERMAPPER_API GlobalGraphMarkerMapper:public MarkerMapper {


public:

    void optimize_impl (    ) ;
     bool process (const arucoMarkerSet &in_markers, int imageIdx=-1  , bool forceConnectedComponent=false) ;
    void drawDetectedMarkers (cv::Mat &imshow , int border_width=1);
    virtual std::string getName()const {return "global_graph";}

    FrameSet getOptimizingFrameSet(FrameSet &fset, MarkerSet &mset, aruco::CameraParameters cam_params, double maxRerpjErr, int minNMarkers);



private:
    std::unordered_set<uint32_t> accepted_markers;
    float max_marker_cam_angle=75; //degs
    float max_angular_dot_value=0.15;
    float radius_search_t=0.1;//translation radius search in kdtree
    std::map<uint32_t,Kdtree> kdtrees;//for each markers, a kdtree with the observed ,stored locations
    arucoMarkerSet detected_markers;

    //----------------
    void compute_pairwise_information(FrameSet &FrameSet , const aruco::CameraParameters &cam_params,StgMatrix<se3> &pose_graph,graph::Graph<int> &cost_graph, std::map<uint32_t,uint32_t>  &marker_id_pos);


    //returns the ith element of a set
    template<typename T,typename T2> T2 at(const std::map<T,T2> &s,int idx){
        auto mid_it=s.begin();
        for(int j=0;j<idx;j++)mid_it++;
        return mid_it->second;
    }






    MarkerSet calculateMarkerLocationsFromExpansionGraph(graph::Floyd & falgo,StgMatrix<se3> &poseGraph,uint32_t origin_id,std::map<uint32_t,uint32_t> &markers_id_pos,std::map<uint32_t,uint32_t> &markers_pos_id);


    std::pair< graph::Graph<int>,StgMatrix<se3> >   extractNodes(graph::Graph<int> &cost_graph,StgMatrix<se3> &poseGraph,const std::vector<uint32_t>  &nodes );


    void  fromStream ( std::istream &str )  ;
    void  toStream ( std::ostream &str )  ;


    //debug
    void map_toStream(std::map<uint32_t,uint32_t> &t,std::ostream &str );
    void map_fromStream(std::map<uint32_t,uint32_t> &t,std::istream &str );
    void debug_save(std::string fp,StgMatrix<se3> &poseGraph, graph::Graph<int> &cost_graph,std::map<uint32_t,uint32_t>   &markers_id_pos,std::map<uint32_t,uint32_t>   &markers_pos_id);
    bool debug_read(std::string fp,StgMatrix<se3> &poseGraph, graph::Graph<int> &cost_graph,std::map<uint32_t,uint32_t>   &markers_id_pos,std::map<uint32_t,uint32_t>   &markers_pos_id);

    int min_num_edges_between_nodes=1;





};
};
#endif
