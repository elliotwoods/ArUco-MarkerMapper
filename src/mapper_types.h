#ifndef _MarkerMap_H
#define _MarkerMap_H
#include <opencv2/core/core.hpp>
#include <aruco/marker.h>
#include "marker_mapper_exports.h"
#include <set>
#include <map>
#include <cstdint>
#include<functional>
namespace aruco_mm{

/**
 * @brief The arucoMarkerSet struct represents the set of markers found in a image. It represents the 2d projections and their ids
 */
struct MARKERMAPPER_API arucoMarkerSet: public std::vector<aruco::Marker>{


    arucoMarkerSet(){}
    arucoMarkerSet(const std::vector<aruco::Marker> &ms):std::vector<aruco::Marker>(ms){}

    bool is(uint32_t id){
        for(auto &m:*this) if (m.id==int(id)) return true;
        return false;
    }

    //find any of this here
    bool is(const std::vector<uint32_t> &ids){
        for(auto id:ids) if (is(  id) ) return true;
        return false;
    }
    //find any of this here
    bool is(const arucoMarkerSet &mids){
        for(auto m:mids) if (is(  m.id) ) return true;
        return false;
    }
    bool is(const std::set<uint32_t> &ids){
        for(auto &m:*this) if ( ids.find( m.id)!=ids.end())return true;
        return false;
    }
    bool are(const std::set<uint32_t> &ids){
        for(auto id:ids) if ( !is(id) ) return false;
        return true;
    }
    aruco::Marker &get(int id){
        for(auto &m:*this)  if (m.id==id) return m;
        throw std::runtime_error("could not found the marrker in the set");
    }
    void remove(uint32_t id){
        auto it=find(begin(),end(),aruco::Marker(id));
        if (it!=end()){erase(it);}
    }
};



/**
 * @brief The FrameInfo struct provides information about a frame (id,pose,2d locations of markers seen)
 */
struct MARKERMAPPER_API FrameInfo {
    cv::Mat rt_c2g;//4x4 matrix from global coordinates to camera
    arucoMarkerSet markers;//set of markers detected by aruco
    int frame_idx;//index of frame in sequence
    FrameInfo(){}
    FrameInfo(int fidx){frame_idx=fidx;};
    FrameInfo(int fidx,const arucoMarkerSet &ams){frame_idx=fidx;markers=ams;}
    FrameInfo(int fidx,const arucoMarkerSet &ams,const cv::Mat &RT){frame_idx=fidx;markers=ams;rt_c2g=RT.clone();}
    FrameInfo(const FrameInfo &fi){rt_c2g=fi.rt_c2g.clone();markers=fi.markers;frame_idx=fi.frame_idx; }
    void  fromStream ( std::istream &str )  ;
    void  toStream ( std::ostream &str ) const ;
    friend std::ostream & operator<<(std::ostream &,const FrameInfo &fi);
};

/**
 * @brief The MarkerInfo struct represents a 3d marker of the environment (id,pose and size)
 */
struct  MARKERMAPPER_API MarkerInfo {
    MarkerInfo(){}
    MarkerInfo(const MarkerInfo &mi){id=mi.id;markerSize=mi.markerSize;rt_g2m=mi.rt_g2m.clone();}
    MarkerInfo ( int Id ){ id=Id;markerSize=-1;}
    MarkerInfo ( int Id,float Size ){ id=Id;markerSize=Size;}
    MarkerInfo ( int Id, cv::Mat RT,float Size ){rt_g2m=RT.clone();id=Id;markerSize=Size;}

    cv::Mat rt_g2m;//marker location rt is global2marker (i.e., transform points from the marker to the global ref system)
    int id;//id
    float markerSize; //size in meters
    //returns marker points in global coordinates
    std::vector<cv::Point3f> get3dPoints(bool applyRt=true)const;
    //returns marker points in global coordinates
    static std::vector<cv::Point3f> get3dPoints(double size);

    void  fromStream ( std::istream &str )  ;
    void  toStream ( std::ostream &str )  ;
    friend std::ostream & operator<<(std::ostream &str,const MarkerInfo &fi);
};


/**
 * @brief The IoHelper struct with io routines for serialization
 */
struct MARKERMAPPER_API IoHelper{

    static void fromStream ( cv::Mat &m,std::istream &str )  ;
    static void toStream ( const  cv::Mat &m,std::ostream &str ) ;
    static void toStream(const aruco::Marker &m,std::ostream &str);
    static void fromStream(  aruco::Marker &m,std::istream &str);

    template<typename Tfirst_second,typename key_type,typename data_type>
    static void fromStream__(Tfirst_second &t,std::istream &str){
        //now, the map
        uint32_t s;
        str.read ( ( char* ) &s,sizeof ( s ) );
        t.clear();
        for ( uint32_t i=0; i<s; i++ ) {
            key_type key;
            str.read ( ( char* ) &key,sizeof ( key ) );
            data_type d;
            d.fromStream ( str );
            t.insert ( std::make_pair ( key,d ) );
        }
    }

    template<typename Tfirst_second>
    static void toStream__(Tfirst_second &t,std::ostream &str ){
        uint32_t s=t.size();
        str.write ( ( char* ) &s ,sizeof ( uint32_t ) );
        for ( auto &x: t) {
            str.write ( ( char* ) &x.first,sizeof ( x.first ) );
             x.second.toStream ( str );
        }

    }
};

//IMPORTANT TYPEDEFS
//A set of frames
class  MARKERMAPPER_API FrameSet: public std::map<uint32_t,FrameInfo>  {

public:
    FrameSet(){}
    void saveToFile(std::string fp);
    void readFromFile(std::string fp);
};
//A set of markers
class  MARKERMAPPER_API MarkerSet:public std::map<uint32_t,MarkerInfo>{

public:
    void saveToFile(std::string fp);
    void readFromFile(std::string fp);

    void saveToStream(std::ostream &fp);
    void readFromStream(std::istream & fp);


};

}
#endif

