//convert a ply generated with openmvg to pcd file
#include <iostream>
#include <fstream>
#include <vector>
#include<opencv2/core/core.hpp>
using namespace std;

float tofcolor(uchar r,uchar g,uchar b){
    float f=0;
    char *c=(char*)&f;
    c[0]=r;
    c[1]=g;
    c[2]=b;
    return f;
}

vector<cv::Vec4f> readPly(string path,bool structure_only)throw (std::exception)
{
ifstream file(path);if (!file) throw std::runtime_error("could not open file:"+path);
//skip until end_header

while(!file.eof())
{
    string line;
    std::getline(file,line);
    if (line.find("end_header")!=std::string::npos) break;
}
//go reading until eof
vector<cv::Vec4f> points;

while(!file.eof())
{
    cv::Vec4f point;
    int r,g,b;
    if ( file>>point[0]>>point[1]>>point[2]>>r>>g>>b){
        bool add=true;
       if (structure_only && (r!=g || g!=b)) add=false;

       if (add){
           point[3]=tofcolor(r,g,b);
            if (r==g && g==b)
                point[3]=tofcolor(255,0,0);
            points.push_back(point);
        }
    }
    else break;

}
return points;
}
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

int main(int argc,char **argv){
    try{
        CmdLineParser cml(argc,argv);
        if (argc==1){cerr<<"in.ply out.pcd [-so structur_only]"<<endl;return -1;}
        auto points=readPly(argv[1],cml["-so"]);
        ofstream filePCD(argv[2]);
        if (!filePCD) throw std::runtime_error("could not open output file");
        filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points.size()<<"\nHEIGHT 1\nPOINTS "<<points.size()<<"\nDATA binary\n";

        filePCD.write((char*)&points[0],points.size()*sizeof(points[0]));

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
