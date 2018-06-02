#ifndef PoseGraphOptimizerH_
#define PoseGraphOptimizerH_
#include "utils/se3.h"
#include "stgmatrix.h"
#include <vector>
#include "graph.h"
namespace aruco_mm
{

struct Pose_Error{
     se3 pose;
    double repj_error;
   inline void toStream(std::ostream &str){
    str.write((char*)this,sizeof(*this));
   }
   inline void fromStream(std::istream &str){
       str.read((char*)this,sizeof(*this));
   }
};

class PoseGraphOptimizer{
public:


    /**
     * @brief optimize
     * @param pose_graph_io
     * @return the best node to do the expansion graph for obtaining the global poses
     */
    uint32_t optimize( StgMatrix<Pose_Error> &pose_graph_io,bool removeOutlierLinks=true,const std::vector<std::string> &nodes_info=std::vector<std::string>());


private:
    std::vector<std::pair< graph::Graph<int>,StgMatrix<se3> > >   getGraphComponents(graph::Graph<int> &cost_graph,StgMatrix<se3> &poseGraph,const graph::Floyd &floyd);
    double get_expansiongraph_error(graph::Floyd   &falgo,graph::Graph<int> &cost_graph,int start_node);
    graph::Graph<int> get_expansion_graph(graph::Graph<int> &cost_graph, graph::Floyd &falgo,int start_node);
    int removeOutlierLinksFromCostGraph(graph::Graph<int> &exp_graph,graph::Graph<int> &cost_graph,double ndevs);
    std::vector< std::vector<uint32_t> > basic_cycles( graph::Graph<int> &cost_graph,graph::Graph<int> &expansion_graph);
    StgMatrix<  se3  > rotational_translational_error_minimization( const std::vector<std::vector<uint32_t> > &basic_cycles, StgMatrix<  se3 >   &_mean_graph, graph::Graph<int> &cost_graph);
    StgMatrix<  se3  >  rotational_error_minimization( const std::vector<std::vector<uint32_t> > &basic_cycles, StgMatrix<  se3 >   &initial_posegraph, const std::vector<std::vector<double> > &basic_cycles_weight);
    StgMatrix<  se3  > translational_error_minimization( const std::vector<std::vector<uint32_t> > &basic_cycles, StgMatrix<  se3 >   &_graph,const std::vector<std::vector<double> > &basic_cycles_weights);
    double getTranslationalError( const std::vector<std::vector<uint32_t> > &basic_cycles,   StgMatrix<  se3 >   &_mean_graph);

    std::vector<std::string> nodes_info;

    /**
     * @brief translational_error_minimization
     * @param basic_cycles
     * @param _mean_graph
     * @return
     */
    template<typename T>
    inline T cacc( const std::vector<T> & v,int s){
        if (size_t(s)<v.size())    return v[s];
        return v[ s%v.size()];

    }
    inline uint32_t toEdge(uint16_t a ,uint16_t b){
        if( a>b)std::swap(a,b);
        uint32_t a_b;
        uint16_t *_a_b_16=(uint16_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    }
    inline std::pair<uint16_t,uint16_t> fromEdge(uint32_t a_b){         uint16_t *_a_b_16=(uint16_t*)&a_b;return std::make_pair(_a_b_16[1],_a_b_16[0]);}

    //optimizes routines
    //10x faster
    //matrix multiplication
    inline void matmul(const float *a,const float *b,float *c){

        c[0]= a[0]*b[0]+ a[1]*b[4]+a[2]*b[8];
        c[1]= a[0]*b[1]+ a[1]*b[5]+a[2]*b[9];
        c[2]= a[0]*b[2]+ a[1]*b[6]+a[2]*b[10];
        c[3]= a[0]*b[3]+ a[1]*b[7]+a[2]*b[11]+a[3];

        c[4]= a[4]*b[0]+ a[5]*b[4]+a[6]*b[8];
        c[5]= a[4]*b[1]+ a[5]*b[5]+a[6]*b[9];
        c[6]= a[4]*b[2]+ a[5]*b[6]+a[6]*b[10];
        c[7]= a[4]*b[3]+ a[5]*b[7]+a[6]*b[11]+a[7];

        c[8]=  a[8]*b[0]+ a[9]*b[4]+a[10]*b[8];
        c[9]=  a[8]*b[1]+ a[9]*b[5]+a[10]*b[9];
        c[10]= a[8]*b[2]+ a[9]*b[6]+a[10]*b[10];
        c[11]= a[8]*b[3]+ a[9]*b[7]+a[10]*b[11]+a[11];
        c[12]=c[13]=c[14]=0;
        c[15]=1;

    }


    //converts a vector of 6 floats (rxryrztxtytz) into a 4x4 matrix
    inline void rodrigues_RTVec2M44(const float * rt,cv::Mat &m){
        m.create(4,4,CV_32F);
        float *rt_44=m.ptr<float>(0);
             float rx=rt[0];
            float ry=rt[1];
            float rz=rt[2];
            float tx=rt[3];
            float ty=rt[4];
            float tz=rt[5];
            float nsqa=rx*rx + ry*ry + rz*rz;
            float a=std::sqrt(nsqa);
            float i_a=a?1./a:0;
            float rnx=rx*i_a;
            float rny=ry*i_a;
            float rnz=rz*i_a;
            float cos_a=cos(a);
            float sin_a=sin(a);
            float _1_cos_a=1.-cos_a;
            rt_44[0] =cos_a+rnx*rnx*_1_cos_a;
            rt_44[1]=rnx*rny*_1_cos_a- rnz*sin_a;
            rt_44[2]=rny*sin_a + rnx*rnz*_1_cos_a;
            rt_44[3]=tx;
            rt_44[4]=rnz*sin_a +rnx*rny*_1_cos_a;
            rt_44[5]=cos_a+rny*rny*_1_cos_a;
            rt_44[6]= -rnx*sin_a+ rny*rnz*_1_cos_a;
            rt_44[7]=ty;
            rt_44[8]= -rny*sin_a + rnx*rnz*_1_cos_a;
            rt_44[9]= rnx*sin_a + rny*rnz*_1_cos_a;
            rt_44[10]=cos_a+rnz*rnz*_1_cos_a;
            rt_44[11]=tz;
            rt_44[12]=rt_44[13]=rt_44[14]=0;
            rt_44[15]=1;

    }

    //converts a vector v of 6 floats (rxryrztxtytz) into a 4x4 matrix

    inline void rodrigues_V2M(const cv::Mat &v,cv::Mat &m){
            assert(v.type()==CV_32F);

            float rx=v.ptr<float>(0)[0];
            float ry=v.ptr<float>(0)[1];
            float rz=v.ptr<float>(0)[2];
            m.create(3,3,CV_32F);

            float nsqa=rx*rx + ry*ry + rz*rz;
            float a=std::sqrt(nsqa);
            float i_a=a?1./a:0;
            float rnx=rx*i_a;
            float rny=ry*i_a;
            float rnz=rz*i_a;
            float cos_a=cos(a);
            float sin_a=sin(a);
            float _1_cos_a=1.-cos_a;

            float *rm=m.ptr<float>(0);
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



    //Given a R matrix 3x3 extract the 3x1 rodrigues vector
    inline void rodrigues_M2V(const cv::Mat &Rin,cv::Mat &rvec){

        assert(Rin.total()==9);
        assert(rvec.total()==3&& rvec.type()==CV_32F);
        memset(rvec.ptr<float>(0),0,3*sizeof(float));

        cv::Matx33d U, Vt;
        cv::Vec3d W;
        double theta, s, c;

        assert(checkRange(Rin, true, NULL, -100, 100) );

        cv::Matx33d R=Rin;
        cv::SVD::compute(R, W, U, Vt);
         R = U*Vt;

        cv::Point3d r(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));

        s = std::sqrt((r.x*r.x + r.y*r.y + r.z*r.z)*0.25);
        c = (R(0, 0) + R(1, 1) + R(2, 2) - 1)*0.5;
        c = c > 1. ? 1. : c < -1. ? -1. : c;
        theta = acos(c);

        if( s < 1e-5 )
        {
            double t;

            if( c > 0 )   r = cv::Point3d(0, 0, 0);
            else
            {
                t = (R(0, 0) + 1)*0.5;
                r.x = std::sqrt(std::max(t,0.));
                t = (R(1, 1) + 1)*0.5;
                r.y = std::sqrt(std::max(t,0.))*(R(0, 1) < 0 ? -1. : 1.);
                t = (R(2, 2) + 1)*0.5;
                r.z = std::sqrt(std::max(t,0.))*(R(0, 2) < 0 ? -1. : 1.);
                if( fabs(r.x) < fabs(r.y) && fabs(r.x) < fabs(r.z) && (R(1, 2) > 0) != (r.y*r.z > 0) )
                    r.z = -r.z;
                theta /= norm(r);
                r *= theta;
            }
        }
        else
        {
            double vth = 1/(2*s);
            vth *= theta;
            r *= vth;
        }

        float *ptr=rvec.ptr<float>(0);
        ptr[0]=r.x;
        ptr[1]=r.y;
        ptr[2]=r.z;

    }

    //computes the inverse of a 4x4 matrix, but only of the 3x3 rotational part
    inline void inverseR(const cv::Mat &RT,cv::Mat &RTinv){
        assert(RT.type()==CV_32F);
        assert(RT.cols==RT.rows && RT.cols==3);
        RTinv.create(3,3,CV_32F);
        //traspose R part
        float *rti=RTinv.ptr<float>(0);
        const float *rt=RT.ptr<float>(0);

        rti[0]=rt[0];rti[1]=rt[3];rti[2]=rt[6];
        rti[3]=rt[1];rti[4]=rt[4];rti[5]=rt[7];
        rti[6]=rt[2];rti[7]=rt[5];rti[8]=rt[8];
    }


    //computes the inverse of a 4x4 matrix, but only of the 3x3 rotational part
    inline void inverseR(  cv::Mat &RT){
        assert(RT.type()==CV_32F);
        assert(RT.cols==RT.rows && RT.cols==3);
        //traspose R part
          float *rt=RT.ptr<float>(0);
        std::swap(rt[1],rt[3]);
        std::swap(rt[2],rt[6]);
        std::swap(rt[5],rt[7]);

    }

    inline void setEye(cv::Mat &R){
        assert(R.type()==CV_32F);
        assert(R.cols==R.rows && R.cols==3);
        float *r=R.ptr<float>(0);
        memset(r,0,9*sizeof(float));
        r[0]=r[4]=r[8]=1;
    }
    inline void m33mult(const cv::Mat &A,const cv::Mat &B,cv::Mat &C){
        assert(C.total()==9 && C.type()==CV_32F);

        float *c=C.ptr<float>(0);
        const float *a=A.ptr<float>(0);
        const float *b=B.ptr<float>(0);
        c[0]= a[0]*b[0]+a[1]*b[3]+a[2]*b[6];
        c[1]= a[0]*b[1]+a[1]*b[4]+a[2]*b[7];
        c[2]= a[0]*b[2]+a[1]*b[5]+a[2]*b[8];

        c[3]= a[3]*b[0]+a[4]*b[3]+a[5]*b[6];
        c[4]= a[3]*b[1]+a[4]*b[4]+a[5]*b[7];
        c[5]= a[3]*b[2]+a[4]*b[5]+a[5]*b[8];

        c[6]= a[6]*b[0]+a[7]*b[3]+a[8]*b[6];
        c[7]= a[6]*b[1]+a[7]*b[4]+a[8]*b[7];
        c[8]= a[6]*b[2]+a[7]*b[5]+a[8]*b[8];


    }
    //computes the inverse of a 4x4 matrix
    void inverseRT(const cv::Mat &M,cv::Mat &Minv){
        assert(Minv.total()==16 && M.type()==CV_32F);
        //transposed R first
        const float *m=M.ptr<float>(0);
        float *minv=Minv.ptr<float>(0);

        minv[0]=m[0]; minv[1]=m[4];minv[2]=m[8];
        minv[4]=m[1]; minv[5]=m[5];minv[6]=m[9];
        minv[8]=m[2]; minv[9]=m[6];minv[10]=m[10];

        //now, translation
        minv[3] =-( minv[0]*m[3]+minv[1]*m[7]+minv[2]*m[11]);
        minv[7] =-( minv[4]*m[3]+minv[5]*m[7]+minv[6]*m[11]);
        minv[11]=-( minv[8]*m[3]+minv[9]*m[7]+minv[10]*m[11]);

        minv[12]=minv[13]=minv[14]=0;
        minv[15]=1;
    }

};

}

#endif
