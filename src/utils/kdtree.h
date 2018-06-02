#ifndef KDTREE_H
#define KDTREE_H
#include "se3.h"
#include "nanoflann.h"
#include <memory>
namespace aruco_mm{


struct  Se3Set:public std::vector<se3>
{

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return   size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline float kdtree_distance(const float *p1, const size_t idx_p2,size_t size) const
    {
        (void)size;
        const float d0=p1[0]-at(idx_p2).rt[0];
        const float d1=p1[1]-at(idx_p2).rt[1];
        const float d2=p1[2]-at(idx_p2).rt[2];
        const float d3=p1[3]-at(idx_p2).rt[3];
        const float d4=p1[4]-at(idx_p2).rt[4];
        const float d5=p1[5]-at(idx_p2).rt[5];
        return d0*d0+d1*d1+d2*d2+d3*d3+d4*d4+d5*d5;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline float kdtree_get_pt(const size_t idx, int dim) const
    {
        return at(idx).rt[dim];
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX &bb) const { (void)bb; return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, Se3Set > ,
        Se3Set,
        6/* dim */
        > __kdtree;
class   Kdtree{
public:
    void build(){
        data=std::make_shared< __kdtree>(6,s3s,10);
        data->buildIndex();
    }

    void add(const se3 &p){        s3s.push_back(p);}
    void add_and_build(const se3 &p){
        s3s.push_back(p);
        data=std::make_shared< __kdtree>(6,s3s,10);
        data->buildIndex();
    }

    std::vector<std::pair<size_t,float> >   radiusSearch(const se3 &p,float d,bool sorted=false){
        std::vector<std::pair<size_t,float> >   ret_matches;
        nanoflann::SearchParams params;
        params.sorted=sorted;
        data->radiusSearch((float * ) & p,d*d, ret_matches, params);
        return ret_matches;
     }


    Se3Set s3s;

 private:
    std::shared_ptr<__kdtree> data;

};





struct  tSet:public std::vector<cv::Point3f>
{

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return   size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline float kdtree_distance(const float *p1, const size_t idx_p2,size_t size) const
    {
        (void)size;
        const float d0=p1[0]-at(idx_p2).x;
        const float d1=p1[1]-at(idx_p2).y;
        const float d2=p1[2]-at(idx_p2).z;
        return d0*d0+d1*d1+d2*d2;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline float kdtree_get_pt(const size_t idx, int dim) const
    {
        return ((float*)&(at(idx)))[dim];
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX &bb) const { (void)bb;return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, tSet > ,
        tSet,
        3/* dim */
        > __kdtreeT;
class   Kdtree3f{
public:
    void build(){
        data=std::make_shared< __kdtreeT>(3,tset,10);
        data->buildIndex();
    }

    void add(const cv::Point3f &p){        tset.push_back(p);}
    void add_and_build(const cv::Point3f &p){
        tset.push_back(p);
        data=std::make_shared< __kdtreeT>(3,tset,10);
        data->buildIndex();
    }

    std::vector<std::pair<size_t,float> >   radiusSearch(const cv::Point3f &p,float d,bool sorted=false){
        std::vector<std::pair<size_t,float> >   ret_matches;
        nanoflann::SearchParams params;
        params.sorted=sorted;
          data->radiusSearch((float * ) & p,d*d, ret_matches, params);
        return ret_matches;
     }

    tSet tset;

 private:
    std::shared_ptr<__kdtreeT> data;

};
}
#endif // KDTREE_H
