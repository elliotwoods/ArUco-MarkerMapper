//Create the files for testing results with VIsualSFM
//after creation, run the following commands to test
//VisualSFM.exe sfm+import . vsfm_res matchesvsfm.txt
//VisualSFM.exe vsfm_res.nvm

#include <markermapper.h>
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
using namespace std;

class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

using namespace std;
using namespace aruco_mm;
//returns the ith element of a set
template<typename T> T at(const std::map<uint32_t,T> &s,int idx){
    auto mid_it=s.begin();
    for(int j=0;j<idx;j++)mid_it++;
    return mid_it->second;
}

std::string to_string_(int i,int n=5){
    std::string number=std::to_string(i);
    while(number.size()!=n) number="0"+number;
    return number;
}


string ton(uint32_t inumber){
    string number=std::to_string(inumber);
    while(number.size()!=5)number="0"+number;
    return number;
}

void saveJsonFile(string outFile,std::shared_ptr<MarkerMapper> mm) throw(std::exception)
{

    ofstream oFile(outFile);
    if (!oFile)throw std::runtime_error("saveJsonFile");
    uint64_t id=2147483649;
    int idx=0;
    cv::Size img_size=mm->getCameraParams().CamSize;
    float *k=mm->getCameraParams().CameraMatrix.ptr<float>(0);
    float *d=mm->getCameraParams().Distorsion.ptr<float>(0);
    oFile<<"{"<<endl;
   oFile<<" \"sfm_data_version\": \"0.2\",\n \"root_path\": \"\",\n \"views\": [\n";
   for(auto frame:mm->getFrameSet()){
       oFile<<"{  \"key\":"<<frame.first<<", \"value\": {   \"ptr_wrapper\": { \"id\":"<<id++<<", \"data\": {\"local_path\": \"/\", \"filename\": \""<< ton(frame.first)<<".jpg\", \"width\": "<<img_size.width<<", \"height\": "<<img_size.height<<", \"id_view\": "<<frame.first<<", \"id_intrinsic\": 0, \"id_pose\": "<<frame.first<<" } } } }";
       if (++idx!=mm->getFrameSet().size()) oFile<<",";
        oFile<<"\n" ;


   }
 oFile<<"],\n" ;
 oFile<<"\"intrinsics\": [ { \"key\": 0, \"value\": {\"polymorphic_id\": 2147483649, \"polymorphic_name\": \"pinhole_radial_k3\", \"ptr_wrapper\": { \"id\": 2147483652, \"data\": {\"width\": "<<img_size.width<<" , \"height\": "<<img_size.height<<", \"focal_length\": "<<k[0]<<" , \"principal_point\": [ "<<k[2]<<" , "<<k[5]<<" ] , \"disto_k3\": [ "<<d[0]<<" , "<<d[1]<<" , "<<d[4]<< " ] } } } } ],\n";
 oFile<< "\"extrinsics\": [], \"structure\": [], \"control_points\": [] \n}\n";

}

void saveFeatFiles(string outDir,std::shared_ptr<MarkerMapper> mm) throw(std::exception){

    for(auto frame:mm->getFrameSet()){
        string fpath=outDir+"/"+ton(frame.first)+".feat";
        ofstream oFile(fpath);
        if (!oFile)throw std::runtime_error("saveFeatFiles error open fle:"+fpath);
        for(auto marker:frame.second.markers)
            for(auto corners:marker)
                oFile<<corners.x<< " "<<corners.y<<" 0 0 "<<endl;
    }

}
void saveMatchesFiles(string outFile,std::shared_ptr<MarkerMapper> armm,float percentage=1) throw(std::exception){

    ofstream oFile(outFile);
    if (!oFile)throw std::runtime_error("saveMatchesFiles");
    std::vector<int> frames;
    for(int i=0;i<armm->getFrameSet().size();i++) frames.push_back(i);
    //remove some of them randomly
    std::random_shuffle(frames.begin(),frames.end());
    if (percentage<1)
        frames.resize(float(frames.size())*percentage);


    for(int i=0;i<frames.size();i++){
        auto frame_i=at(armm->getFrameSet(),frames[i]);
        for(int j=i+1;j<frames.size();j++){
            auto frame_j=at(armm->getFrameSet(),frames[j]);

            //check matches
            vector<std::pair<int,int> > matches;
            for(int mi=0;mi<frame_i.markers.size();mi++)
                for(int mj=0;mj<frame_j.markers.size();mj++)
                    if ( frame_i.markers[mi].id==frame_j.markers[mj].id)
                        for(int m=0;m<4;m++) matches.push_back(make_pair(mi*4+m,mj*4+m));
            //save info
            if ( matches.size()>0){
                oFile<<frame_i.frame_idx <<" "<< frame_j.frame_idx <<endl<<matches.size()<<endl;
                for(auto m:  matches )  oFile<<m.first<<" "<<m.second<<endl;
            }
        }
    }
}




int main(int argc,char **argv){

    try{
        CmdLineParser cml(argc,argv);
        if (argc==1){cerr<<"Usage: in.amm outdir [-p percentage_matches]"<<endl; return -1;}
        {
            stringstream cmd;cmd<<"mkdir "<<argv[2]<<" -p";
            system(cmd.str().c_str());
        }
        {
            stringstream cmd;cmd<<"rm "<<argv[2]<<"/* -rf";
            system(cmd.str().c_str());
        }
        auto armm=aruco_mm::MarkerMapper::readFromFile(argv[1]);
        saveJsonFile(string(argv[2])+"/sfm_data.json",armm);
        saveFeatFiles(argv[2],armm);
        saveMatchesFiles(string(argv[2])+"/matches.e.txt",armm,std::stof(cml("-p","1.0")));
        saveMatchesFiles(string(argv[2])+"/matches.f.txt",armm,std::stof(cml("-p","1.0")));
        cerr<<"Done, now run"<<endl;
        cerr<<"   openMVG_main_GlobalSfM -i "<<argv[2]<<"/sfm_data.json -m " <<argv[2]<< " -o "<<argv[2]<<"/reconstruction/"<<endl;
    }catch(std::exception &ex){
        cout<<ex.what()<<endl;
    }
}
