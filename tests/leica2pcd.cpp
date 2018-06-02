#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>

using namespace std;

float tofcolor(uchar r,uchar g,uchar b){
    float f=0;
    char *c=(char*)&f;
    c[0]=r;
    c[1]=g;
    c[2]=b;
    return f;
}

int main(int argc,char **argv){

    if (argc!=3){cerr<<"USage in.txt out.pcd"<<endl;return -1;}

       vector<cv::Vec4f> points;
       std::ifstream file(argv[1]);
       if (!file){cerr<<"Could not open input"<<endl;return -1;}
       std::ofstream filePCD(argv[2]);
       if (!filePCD){cerr<<"Could not open output"<<endl;return -1;}
       //skip first line
       string str;std::getline(file,str);
       int it=0;
       while(!file.eof() ){
           if(it%100)cerr<<it<<" ";
           std::getline(file,str);
           //remove commas
           for(auto &c:str)if (c==',')c=' ';
           stringstream sline;sline<<str;
            cv::Vec4f p;
            int r,g,b,i;
            if ( sline>>p[0]>>p[1]>>p[2]>>r>>g>>b>>i){
                p[3]=tofcolor(r,g,b);
                points.push_back(p);

            }
            it++;
       }

       filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points.size()<<"\nHEIGHT 1\nPOINTS "<<points.size()<<"\nDATA binary\n";

       filePCD.write((char*)&points[0],points.size()*sizeof(points[0]));


}
