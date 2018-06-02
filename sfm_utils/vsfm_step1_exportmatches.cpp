//Create the files for testing results with VIsualSFM
//after creation, run the following commands to test
//VisualSFM.exe sfm+import . vsfm_res matchesvsfm.txt
//VisualSFM.exe vsfm_res.nvm

#include <markermapper.h>
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>

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

cv::Point2f max(cv::Point2f  a,cv::Point2f  b){
    return cv::Point2f( max(a.x,b.x), max(a.y,b.y));
}

cv::Point2f  saveFrameInfo(const aruco_mm::FrameInfo &frame,string outdir){
    ofstream file(outdir+"/frame-"+to_string_(frame.frame_idx)+".sift");
    file<<frame.markers.size()*4<<" 128\n";
    cv::Point2f max_p(0,0);
    for(auto marker:frame.markers){
        for(auto point:marker){
            file<<point.x<<" "<<point.y<<" 1.0 1.0\n";
            max_p=max(point,max_p );
            for(int k=0; k<6; k++) file << "1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1\n";
                     file << "1 1 1 1 1 1 1 1\n";
        }
    }
    return max_p;
}

int main(int argc,char **argv){

    try{
        if (argc!=3){cerr<<"Usage: in.amm outdir "<<endl;return -1;}
        auto armm=aruco_mm::MarkerMapper::readFromFile(argv[1]);
        ofstream matches_file(argv[2]+string("/matchesvsfm.txt"));
        cv::Point2f max_p(0,0);

        size_t max_frames=std::numeric_limits<size_t>::max();
        for(int i=0;i<min(max_frames,armm->getFrameSet().size());i++){
            cerr<<i<<" ";
            auto frame_i=at(armm->getFrameSet(),i);
            auto max_p_i=saveFrameInfo(frame_i,argv[2]);
            max_p= max(max_p_i,max_p );
            for(int j=i+1;j<min(max_frames,armm->getFrameSet().size());j++){
                auto frame_j=at(armm->getFrameSet(),j);

                //check matches
                vector<std::pair<int,int> > matches;
                for(int mi=0;mi<frame_i.markers.size();mi++)
                    for(int mj=0;mj<frame_j.markers.size();mj++)
                        if ( frame_i.markers[mi].id==frame_j.markers[mj].id)
                            for(int m=0;m<4;m++) matches.push_back(make_pair(mi*4+m,mj*4+m));
                //save info
                if ( matches.size()>0){
                    matches_file<<"frame-"<<to_string_(frame_i.frame_idx)<<".jpg frame-"<<to_string_(frame_j.frame_idx)<<".jpg "<<matches.size()<<endl;
                    for(auto m:  matches )  matches_file<<m.first<<" ";matches_file<<endl;
                    for(auto m:  matches )  matches_file<<m.second<<" ";matches_file<<endl;
                }
            }
        }
        matches_file.close();
        //now, create the images as empty
        cv::Mat img( armm->getCameraParams().CamSize,CV_8UC1);
        img.setTo(cv::Scalar::all(0));
        cerr<<endl<<"saving images"<<endl;
        for(int i=0;i<min(max_frames,armm->getFrameSet().size());i++){
            auto frame=at(armm->getFrameSet(),i);
            cerr<<frame.frame_idx<<" ";
            string name=argv[2]+string("/frame-")+to_string_(frame.frame_idx)+".jpg";
            cv::imwrite(name,img);
        }

        cout<<"VisualSFM.exe sfm+import . vsfm_res matchesvsfm.txt"<<endl;
        cout<<"VisualSFM.exe vsfm_res.nvm"<<endl;

    }catch(std::exception &ex){
        cout<<ex.what()<<endl;
    }
}
