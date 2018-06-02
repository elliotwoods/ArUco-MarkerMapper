
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
int main(){
    cv::VideoCapture vcap(0);
    if (!vcap.isOpened()){std::cerr<<"no cam"<<std::endl;return -1;};
    cv::Mat image;
    while(!image.empty()) vcap>>image;
    char key=0;

    cv::VideoWriter outputVideo;                                        // Open the output
    outputVideo.open("out.avi",  -1,30, image.size(), true);
    if (!outputVideo.isOpened()){std::cerr<<"no vout"<<std::endl;return -1;};

    while(key!=27){

        vcap>>image;
        outputVideo<<image;
        cv::imshow("image",image);
        key=cv::waitKey(10);
    }
}
