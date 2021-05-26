#include "mapper_types.h"
#include "utils/utils3d.h"
#include <fstream>
namespace aruco_mm{




std::vector<cv::Point3f> MarkerInfo::get3dPoints(bool applyRt)const{//returns marker points in global coordinates
    float size_2=markerSize/2.;
    //now, that the current location is estimated, add new markers and update old ones
     std::vector<cv::Point3f> points = { cv::Point3f ( -size_2, size_2,0 ),cv::Point3f ( size_2, size_2 ,0 ),
                                   cv::Point3f ( size_2, -size_2,0 ),cv::Point3f ( -size_2, -size_2,0 )  };
    if (!rt_g2m.empty() && applyRt)   aruco_mm::mult<cv::Point3f>(rt_g2m,points);

    return points;
}
std::vector<cv::Point3f> MarkerInfo::get3dPoints(double size){//returns marker points in global coordinates
     float size_2=size/2.;
    //now, that the current location is estimated, add new markers and update old ones
    return { cv::Point3f ( -size_2, size_2,0 ),cv::Point3f ( size_2, size_2 ,0 ),
                                   cv::Point3f ( size_2, -size_2,0 ),cv::Point3f ( -size_2, -size_2,0 )  };
}




//// --------------------------------------------------------
/////          SERIALIZATION ROUTINES
//// --------------------------------------------------------
void IoHelper::toStream(const aruco::Marker &m,std::ostream &str){
    uint32_t s=m.size();
    str.write((char*)&s,sizeof(s));
    str.write((char*)&m[0],sizeof(m[0])*m.size());
    str.write((char*)&m.id,sizeof(m.id));
    str.write((char*)&m.ssize,sizeof(m.ssize));
    toStream(m.Rvec,str);
    toStream(m.Tvec,str);
}
void  IoHelper::fromStream(aruco::Marker &m,std::istream &str){
    uint32_t s;
    str.read((char*)&s,sizeof(s));
    m.resize(s);
    str.read((char*)&m[0],sizeof(m[0])*m.size());
    str.read((char*)&m.id,sizeof(m.id));
    str.read((char*)&m.ssize,sizeof(m.ssize));
    fromStream(m.Rvec,str);
    fromStream(m.Tvec,str);
}

void  IoHelper::toStream ( const  cv::Mat &m,std::ostream &str ) {

    str.write ( ( char* ) &m.rows,sizeof ( int ) );
    str.write ( ( char* ) &m.cols,sizeof ( int ) );
    int t=m.type();
    str.write ( ( char* ) &t,sizeof ( int ) );
    //write data row by row
    for ( int y=0; y<m.rows; y++ )
        str.write ( m.ptr<char> ( y ),m.cols *m.elemSize() );
}
/**
 */

void  IoHelper::fromStream ( cv::Mat &m,std::istream &str ) {
    int r,c,t;
    str.read ( ( char* ) &r,sizeof ( int ) );
    str.read ( ( char* ) &c,sizeof ( int ) );
    str.read ( ( char* ) &t,sizeof ( int ) );
    m.create ( r,c,t );
    for ( int y=0; y<m.rows; y++ )
        str.read ( m.ptr<char> ( y ),m.cols *m.elemSize() );
}



void FrameInfo::toStream(std::ostream &str)const
{
    IoHelper::toStream(rt_c2g,str);
    uint32_t size=markers.size();
    str.write((char*)&size,sizeof(uint32_t));
    //write the points
    for(auto &m:markers)  IoHelper::toStream(m,str);
    str.write((char*)&frame_idx,sizeof(int));
}



void FrameInfo::fromStream(std::istream &str)
{
    IoHelper::fromStream(rt_c2g,str);
    uint32_t size;
    str.read((char*)&size,sizeof(uint32_t));
    markers.resize(size);
    for(auto &m:markers)  IoHelper::fromStream(m,str);
    str.read((char*)&frame_idx,sizeof(int));
}


void MarkerInfo::toStream(std::ostream &str){
   IoHelper:: toStream(rt_g2m,str);
    str.write((char*)&id,sizeof(int));
    str.write((char*)&markerSize,sizeof(float));
}
void MarkerInfo::fromStream(std::istream &str){
    IoHelper::fromStream(rt_g2m,str);
    str.read((char*)&id,sizeof(int));
    str.read((char*)&markerSize,sizeof(float));
}
void MarkerSet::saveToFile(std::string fp){

    std::ofstream file(fp,std::ios::binary);
    if (!file) throw std::runtime_error("Could not open file for writing:"+fp);
    saveToStream(file);

}

void MarkerSet::readFromFile(std::string fp){
    std::ifstream file(fp,std::ios::binary);
    if (!file) throw std::runtime_error("Could not open file for reading:"+fp);
    readFromStream(file);

}


void MarkerSet::saveToStream(std::ostream &str){
    uint64_t sig=222237123;
    str.write((char*)&sig,sizeof(sig));
    IoHelper::toStream__(*this,str);

}

void MarkerSet::readFromStream(std::istream & str)
{
    uint64_t sig=0;
    str.read((char*)&sig,sizeof(sig));
    if (sig!=222237123)throw std::runtime_error("File   is not of approrpiate type");
    IoHelper::fromStream__<MarkerSet,uint32_t,MarkerInfo>(*this,str);

}
void  FrameSet::saveToFile(std::string fp){
    std::ofstream file(fp,std::ios::binary);
    if (!file) throw std::runtime_error("Could not open file for writing:"+fp);

    uint32_t s=size();
    file.write((char*)&s,sizeof(s));
    for(const auto &e:*this){
        file.write((char*)&e.first,sizeof(e.first));
        e.second.toStream(file);
    }
}

void FrameSet::readFromFile(std::string fp){
    std::ifstream file(fp,std::ios::binary);
    if (!file) throw std::runtime_error("Could not open file for reading:"+fp);

    clear();
    uint32_t s;
    file.read((char*)&s,sizeof(s));
    for(uint32_t i=0;i<s;i++ ){
        uint32_t first;
        FrameInfo second;

        file.read((char*)&first,sizeof(first));
        second.fromStream(file);

        insert({first,second});
    }
}


}
