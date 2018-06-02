#ifndef s3_arucomm_h
#define s3_arucomm_h
#include <opencv2/calib3d/calib3d.hpp>
#include "utils3d.h"
#include <iostream>
#include <vector>
namespace aruco_mm{

class  se3{
public:
    float rt[6];
    se3(){  for(int i=0;i<6;i++) rt[i]=std::numeric_limits<float>::quiet_NaN();}
    se3(float rx,float ry,float rz,float tx,float ty,float tz){   rt[0]=rx;rt[1]=ry;rt[2]=rz;rt[3]=tx;rt[4]=ty;rt[5]=tz;}
    se3(float v){  for(int i=0;i<6;i++) rt[i]=v;}
    se3(const cv::Mat &rt){  *this=convert(rt);}
    se3(const cv::Mat &r,const cv::Mat &t){  *this=convert(r,t);}
    bool isValid(){for(int i=0;i<6;i++)  if ( std::isnan(rt[i]))return false;  return true;}
    void operator+=(const se3 &s){for(int i=0;i<6;i++) rt[i]+=s.rt[i];}
    float &operator()(int i){ return rt[i];}
    void operator/=(float val){for(int i=0;i<6;i++) rt[i]/=val;}
    se3 & operator*(float val){for(int i=0;i<6;i++) rt[i]*=val;return *this;}
    se3   operator*(const se3  &b)const{ return se3(convert()*b.convert());}
    se3 & operator=(const cv::Mat &rt){ *this=convert(rt);return *this;}
    void setTo(float v){for(int i=0;i<6;i++) rt[i]=v;}
    float operator[](size_t d)const {return rt[d];}
    float &operator[](size_t d){return rt[d];}
    friend std::ostream & operator<<(std::ostream &str,const se3 &s){for(int i=0;i<6;i++)str<<s.rt[i]<<" ";return str;}
    operator cv::Mat (){return convert();}


    float r_dot(const se3 &b){
        cv::Vec3f ra(rt[0],rt[1],rt[2]);
        cv::Vec3f rb(b.rt[0],b.rt[1],b.rt[2]);
        ra*=1./cv::norm(ra);
        rb*=1./cv::norm(rb);
        return ra.dot(rb);
    }
    float t_dist(const se3 &b){
        double s=0;
        for(int i=3;i<6;i++) s+=(rt[i]-b.rt[i] )*(rt[i]-b.rt[i] );
        return sqrt(s);
    }

    float tnorm()const{
        return sqrt( rt[3]*rt[3]+ rt[4]*rt[4]+rt[5]*rt[5]);
    }

    void setRotation(float rx,float ry,float rz){
        rt[0]=rx;
        rt[1]=ry;
        rt[2]=rz;
    }
    void setRotation(const cv::Mat &rvec){
        if (rvec.type()==CV_32F)
            memcpy( rt,rvec.ptr<float>(0),3*sizeof(float) );
        else if ( rvec.type()==CV_64F)
            memcpy( rt,rvec.ptr<double>(0),3*sizeof(double) );
        else{
            assert(0);
        }
    }


    inline void getRotation3x3(cv::Mat &R){

                 assert(R.type()==CV_32F);

                float rx=rt[0];
                float ry=rt[1];
                float rz=rt[2];
                R.create(3,3,CV_32F);

                float nsqa=rx*rx + ry*ry + rz*rz;
                float a=std::sqrt(nsqa);
                float i_a=a?1./a:0;
                float rnx=rx*i_a;
                float rny=ry*i_a;
                float rnz=rz*i_a;
                float cos_a=cos(a);
                float sin_a=sin(a);
                float _1_cos_a=1.-cos_a;

                float *rm=R.ptr<float>(0);
                rm[0] =cos_a+rnx*rnx*_1_cos_a;
                rm[1]=rnx*rny*_1_cos_a- rnz*sin_a;
                rm[2]=rny*sin_a + rnx*rnz*_1_cos_a;
                rm[3]=rnz*sin_a +rnx*rny*_1_cos_a;
                rm[4]=cos_a+rny*rny*_1_cos_a;
                rm[5]= -rnx*sin_a+ rny*rnz*_1_cos_a;
                rm[6]= -rny*sin_a + rnx*rnz*_1_cos_a;
                rm[7]= rnx*sin_a + rny*rnz*_1_cos_a;
                rm[8]=cos_a+rnz*rnz*_1_cos_a;



    }

    cv::Mat getRotation3x3() {//as a 3x3 matrix
        cv::Mat rot;
        cv::Mat r3(1,3,CV_32F,rt);
        cv::Rodrigues(r3,rot);
        return rot;

    }
    cv::Mat getTvec() {//as a 3x3 matrix
        cv::Mat m(1,3,CV_32F);
        memcpy(m.ptr<float>(0),rt+3,3*sizeof(float));
        return m;
    }
    cv::Mat getRvec() {//as a 3x3 matrix
        cv::Mat m(1,3,CV_32F);
        memcpy(m.ptr<float>(0),rt,3*sizeof(float));
        return m;
    }
    void setTranslation(const cv::Mat &tvec){

        if (tvec.type()==CV_32F)
            memcpy( rt+3,tvec.ptr<float>(0),3*sizeof(float) );
        else if ( tvec.type()==CV_64F)
            memcpy( rt+3,tvec.ptr<double>(0),3*sizeof(double) );
        else{
            assert(0);
        }
    }
    se3  convert(const  cv::Mat &r,const cv::Mat &t) {
        assert(r.type()==CV_32F);
        se3 res;
        for(int i=0;i<3;i++){
            res.rt[i]=r.ptr<float>(0)[i];
            res.rt[i+3]=t.ptr<float>(0)[i];
        }
        return res;
    }

    static se3  convert(const  cv::Mat &RT){

        se3 res;
        if (RT.empty())return res;
        cv::Mat r,t;
        aruco_mm::getRTfromMatrix44(RT,r,t);
        if ( RT.type()==CV_32F){
            for(int i=0;i<3;i++){
                res.rt[i]=r.ptr<float>(0)[i];
                res.rt[i+3]=t.ptr<float>(0)[i];
            }
        }
        else{
            for(int i=0;i<3;i++){
                res.rt[i]=r.ptr<double>(0)[i];
                res.rt[i+3]=t.ptr<double>(0)[i];
            }

        }
        return res;
    }

    cv::Mat   convert( )const{
        float *aa=(float*)rt;
        cv::Mat r(1,3,CV_32F,  aa);
        cv::Mat t (1,3,CV_32F, aa+3);
        return aruco_mm::getRTMatrix(r,t);
    }

    se3 inverse( ){return convert(this->convert().inv());}


    void toStream(std::ostream &str){str.write((char*)rt,6*sizeof(float));}
    void fromStream(std::istream &str){str.read((char*)rt,6*sizeof(float));}

};
}
#endif
