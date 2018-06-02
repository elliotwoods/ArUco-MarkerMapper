#include <markermapper.h>
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
//Bundler.exe imagelist.txt --options_file options.txt
//rm  constraints.txt bundle.out matches.corresp.txt matches.prune.txt matches.ransac.txt nmatches.corresp.txt nmatches.prune.txt nmatches.ransac.txt

//bundler_ply2pcd
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
    ofstream file(outdir+"/frame-"+to_string_(frame.frame_idx)+".key");
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
void write_options_file(string outfile){
    ofstream file(outfile);
    file<<"--run_bundle"<<endl;
      file<<"--match_table matches_bundler.txt"<<endl;
      file<<"--output bundle.out"<<endl;
      file<<"--constrain_focal"<<endl;
      file<<"--use_focal_estimate"<<endl;
      file<<"--estimate_distortion"<<endl;
      file<<"--variable_focal_length"<<endl;
     file<<"--constrain_focal_weight 0.0001"<<endl;

}


void writeIntrinsicFile( string outfile,std::shared_ptr<aruco_mm::MarkerMapper> armm ) {
    ofstream intrinsicsfile(outfile);
    auto _camMatrix=armm->getCameraParams().CameraMatrix;
    auto _distCoeffs=armm->getCameraParams().Distorsion;
    CV_Assert(_camMatrix.total() == 9);
    CV_Assert(_distCoeffs.total() <= 5);

    intrinsicsfile << armm->getFrameSet().size()<< "\n";
    for(int c=0; c<armm->getFrameSet().size(); c++) {
        cv::Mat aux;
        _camMatrix.convertTo(aux, CV_64FC1);
        for(int i=0; i<aux.total(); i++)
            intrinsicsfile << aux.ptr<double>()[i] << " ";
        intrinsicsfile << "\n";

        _distCoeffs.convertTo(aux, CV_64FC1);
        for(int i=0; i<aux.total(); i++)
            intrinsicsfile << aux.ptr<double>()[i] << " ";
        for(int i=aux.total(); i<5; i++)
            intrinsicsfile << "0.0 ";
        intrinsicsfile << "\n";
    }
    intrinsicsfile.close();
}


void writeImageList(string outfile,std::shared_ptr<aruco_mm::MarkerMapper> armm ,bool fixIntrisics){
    ofstream file(outfile);
    auto _camMatrix=armm->getCameraParams().CameraMatrix;
    double param=0.5*(_camMatrix.ptr<float>()[0]+_camMatrix.ptr<float>()[4]);
    for(auto frame:armm->getFrameSet())
        if (fixIntrisics)
            file<<"frame-"+to_string_(frame.first)<<".jpg 0 "<<param<<endl;
    else
            file<<"frame-"+to_string_(frame.first)<<".jpg "<<endl;
}

int main(int argc,char **argv){

    try{
        if (argc!=3){cerr<<"Usage: in.amm outdir "<<endl;return -1;}
       auto armm =aruco_mm::MarkerMapper::readFromFile(argv[1]);

        if (0){//reduce
            size_t max_frames=120;
            aruco_mm::FrameSet fs;
            for(int i=0;i<min(max_frames,armm->getFrameSet().size());i++){
                aruco_mm::FrameInfo frame=at(armm->getFrameSet(),i);
                fs.insert(std::make_pair(frame.frame_idx,frame) );
            }
            armm->getFrameSet()=fs;
        }
        write_options_file(argv[2]+string("/options.txt"));
        writeIntrinsicFile(argv[2]+string("/intrinsics.txt"),armm);
        writeImageList(argv[2]+string("/imagelist.txt"),armm,false);

        ofstream matches_file(argv[2]+string("/matches_bundler.txt"));
        cv::Point2f max_p(0,0);

        //size_t max_frames=std::numeric_limits<size_t>::max();
        for(int i=0;i< armm->getFrameSet().size();i++){
            cerr<<i<<" ";
            auto frame_i=at(armm->getFrameSet(),i);
            auto max_p_i=saveFrameInfo(frame_i,argv[2]);
            max_p= max(max_p_i,max_p );
            for(int j=i+1;j<armm->getFrameSet().size();j++){
                auto frame_j=at(armm->getFrameSet(),j);

                //check matches
                vector<std::pair<int,int> > matches;
                for(int mi=0;mi<frame_i.markers.size();mi++)
                    for(int mj=0;mj<frame_j.markers.size();mj++)
                        if ( frame_i.markers[mi].id==frame_j.markers[mj].id)
                            for(int m=0;m<4;m++) matches.push_back(make_pair(mi*4+m,mj*4+m));
                //save info
                if ( matches.size()>0){
                    matches_file<<i<< " "<<j<<endl<<matches.size()<<endl;
                    for(auto m:  matches )  matches_file<<m.first<<" "<<m.second<<endl;
                }
            }
        }
        matches_file.close();
        //now, create the images as empty
        cv::Mat img( armm->getCameraParams().CamSize,CV_8UC1);
        img.setTo(cv::Scalar::all(0));
        cerr<<endl<<"saving images"<<endl;
        for(int i=0;i< armm->getFrameSet().size();i++){
            auto frame=at(armm->getFrameSet(),i);
            cerr<<frame.frame_idx<<" ";
            string name=argv[2]+string("/frame-")+to_string_(frame.frame_idx)+".jpg";
            cv::imwrite(name,img);
        }
        cout<<"\n\nNow run \n\tcd "<<argv[2]<<"\n\tBundler.exe imagelist.txt --options_file options.txt"<<endl;
    }catch(std::exception &ex){
        cout<<ex.what()<<endl;
    }
}
