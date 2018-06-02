#include <iostream>
#include <opencv2/highgui.hpp>

#include <aruco/markermap.h>
#include <aruco/markerdetector.h>
#include <aruco/posetracker.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
using namespace std;

class Game{

    struct Cube{
      cv::Point3f center=cv::Point3f(0,0,0);
      float size;
      std::vector<cv::Point3f> getCubeCorners()const {
            vector<cv::Point3f> corners={
              cv::Point3f(0,0,0),cv::Point3f(size,0,0),cv::Point3f(size,size,0),cv::Point3f(0,size,0),
              cv::Point3f(0,0,size),cv::Point3f(size,0,size),cv::Point3f(size,size,size),cv::Point3f(0,size,size)
            };
            for(auto &c:corners)
                c+=center;

            return corners;

      }
    };

    aruco::MarkerMap _mmap;
    aruco::CameraParameters _camParams;

    aruco::MarkerDetector mdetector;
    aruco::MarkerMapPoseTracker PoseTracker;
    Cube _cube;
    bool isInited=false;
public:
    void setParams(const aruco::CameraParameters &camParams,const aruco::MarkerMap &mmap ) {

        _camParams=camParams;
        _mmap=mmap;
        //put the Cube in the center of the scene
       _cube.size=mmap[0].getMarkerSize();
       mdetector.setDictionary(mmap.getDictionary());
    }

    void translateCube(float tx,float ty,float tz){
        _cube.center+=cv::Point3f(tx,ty,tz);
    }

    void draw(  cv::Mat &image){
        if (!isInited){
            isInited=true;
            _camParams.resize(image.size());
            PoseTracker.setParams(_camParams,_mmap);
        }
        //find the projection of the cube corners
        auto markers=mdetector.detect(image);
        if (PoseTracker.estimatePose(markers)){
            auto corners=_cube.getCubeCorners();
            //project on image
            vector<cv::Point2f> points2d;
            cv::projectPoints(corners,PoseTracker.getRvec(),PoseTracker.getTvec(),_camParams.CameraMatrix,_camParams.Distorsion,points2d);
            //now, draw lines between them
            for(int i=0;i<4;i++){
                cv::line(image,points2d[i],points2d[(i+1)%4],cv::Scalar(0,255,0),2);
                cv::line(image,points2d[4+i],points2d[4+(i+1)%4],cv::Scalar(255,0,0),2);
                cv::line(image,points2d[i],points2d[(i+4)],cv::Scalar(0,0,255),2);
            }
            //draw markers too
            for(auto m:markers)
                m.draw(image);
        }
    }

};


int main(int argc,char **argv){

    try{
        if (argc!=4){
            cerr<<"Usage: cameraIndex map cameraParams"<<endl;
            return -1;
        }
        cv::VideoCapture camera(std::stoi(argv[1]));
        if(!camera.isOpened())throw std::runtime_error("Could not open camera");

        aruco::MarkerMap mmap;
        mmap.readFromFile(argv[2]);
        aruco::CameraParameters cameraParams;
        cameraParams.readFromXMLFile(argv[3]);
        char key=0;
        cv::Mat inimage,image;
        Game TheGame;
        TheGame.setParams(cameraParams,mmap);
        while(camera.grab() && key!=27){

                camera.retrieve(image);
                TheGame.draw(image);

                cv::cvtColor(image,inimage,CV_BGR2BGRA);
                cv::imshow("image",inimage);
                key=cv::waitKey(4);
                if (key!=-1)
                    cout<<(int)key<<endl;
                if (key==81)
                    TheGame.translateCube(-0.05,0,0);
                if (key==83)
                    TheGame.translateCube(0.05,0,0);
                if (key==82)
                    TheGame.translateCube(0,-0.05,0);
                if (key==84)
                    TheGame.translateCube(0,0.05,0);
        }


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
