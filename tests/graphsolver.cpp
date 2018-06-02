#include <Eigen/Core>
#include <iostream>
#include<memory>
namespace graphsolver{


struct Vertex{
    virtual Eigen::VectorXd  getMeasure()const=0;
};


struct Edge{
    virtual Eigen::VectorXd  getErrorMeasure()const=0;

    inline void addEdge(std::shared_ptr<Edge>  edge){edges.push_back(edge);}
protected:
    std::vector<std::shared_ptr<Edge> > edges;
};


struct Graph{

    inline void addVertex( std::shared_ptr<Vertex> v ){ vertices.push_back(v);}

    void optimize(){}
    protected:
        std::vector<std::shared_ptr<Vertex> > vertices;
};

};

struct Point3d:public graphsolver::Vertex {
    Eigen::VectorXd _point;
public:
    Point3d(float x,float y,float z){
        _point.resize(3);
        _point(0)=x;
        _point(1)=y;
        _point(2)=z;
    }
    inline Eigen::VectorXd  getMeasure()const{return _point;};
};

struct CameraPose:public  graphsolver::Vertex {

    Eigen::Matrix4d _poseMatrix;
    Eigen::VectorXd _poseVector;
    CameraPose(float rx,float ry,float rz,float tx,float ty,float tz){
        _poseVector.resize(6);
        _poseVector(0)=rx;
        _poseVector(1)=ry;
        _poseVector(2)=rz;
        _poseVector(3)=tx;
        _poseVector(4)=ty;
        _poseVector(5)=tz;

        //Creates the 4x4 matrix transform from the rotation and translation components
        float nsqa=rx*rx + ry*ry + rz*rz;
        float a=std::sqrt(nsqa);
        float i_a=a?1./a:0;
        float rnx=rx*i_a;
        float rny=ry*i_a;
        float rnz=rz*i_a;
        float cos_a=cos(a);
        float sin_a=sin(a);
        float _1_cos_a=1.-cos_a;
        _poseMatrix(0,0) =cos_a+rnx*rnx*_1_cos_a;
        _poseMatrix(0,1)=rnx*rny*_1_cos_a- rnz*sin_a;
        _poseMatrix(0,2)=rny*sin_a + rnx*rnz*_1_cos_a;
        _poseMatrix(1,0)=rnz*sin_a +rnx*rny*_1_cos_a;
        _poseMatrix(1,1)=cos_a+rny*rny*_1_cos_a;
        _poseMatrix(1,2)= -rnx*sin_a+ rny*rnz*_1_cos_a;
        _poseMatrix(2,0)= -rny*sin_a + rnx*rnz*_1_cos_a;
        _poseMatrix(2,1)= rnx*sin_a + rny*rnz*_1_cos_a;
        _poseMatrix(2,2)=cos_a+rnz*rnz*_1_cos_a;
        _poseMatrix(0,3)=tx;
        _poseMatrix(1,3)=ty;
        _poseMatrix(2,3)=tz;
        _poseMatrix(3,0)=_poseMatrix(3,1)=_poseMatrix(3,2)=0;
        _poseMatrix(3,3)=1;
    }

    inline Eigen::VectorXd  getMeasure()const{return _poseVector;};
};

struct PointProjection:public graphsolver::Edge{

    float _fx,_fy,_cx,_cy;
    PointProjection(float fx,float fy,float cx,float cy,float k1=0,float k2=0,float p1=0,float p2=0,float k3=0){
        _fx=fx;
        _fy=fy;
        _cx=cx;
        _cy=cy;
    }

    inline Eigen::VectorXd  getErrorMeasure()const{
        //move point using camera pose
        auto *Point=(Point3d*)  edges[0].get();
        auto *Camera=(CameraPose*)  edges[1].get();
        auto p3d_incam = Camera->_poseMatrix* Eigen::Vector4d (Point->_point(0),Point->_point(1),Point->_point(2),1);
        //now, project
        return Eigen::Vector2d ( _cx+ _fx*(p3d_incam(0) /p3d_incam(2)) ,_cy+_fy* (p3d_incam(1) /p3d_incam(2)) );
    };
};


int main(){

    graphsolver::Graph MyGraph;
//    MyGraph.


}
