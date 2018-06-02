#include <iostream>
#include "markermapper.h"
#include "optimizers/fullsceneoptimizer.h"
#include "optimizers/markersetlagragianoptimizer.h"
#include "optimizers/sparselevmarq.h"
#include "utils/utils3d.h"
#include <cmath>
using namespace std;
class Adapter{
public:
    void readFromFile(string fp)throw (std::exception){
        mm=aruco_mm::MarkerMapper::readFromFile(fp);
    }

    virtual std::vector<float>  getInitialSolution( )=0;
    virtual double  evaluate_fitness(const std::vector<float>&sol )=0;
    virtual void saveSolutionToPCD(string fp,const std::vector<float>&sol)=0;
    virtual     std::vector<float> localsearch(const std::vector<float>&sol,int niters,bool verbose)=0;
    //rturns the number of variables of the solution vector that form part of the structure (markers)
    virtual int getStructureSolSize()const = 0;
    //indicates how many variables comprise an structure element
    virtual int getStrutureSize()const=0;


    std::shared_ptr<aruco_mm::MarkerMapper> mm;

};
class MarkerAdapter:public Adapter{
public:


    std::vector<float>  getInitialSolution( ){
         aruco_mm::FullSceneOptimizer opt;
         return opt.getSolution(mm->getMarkerSet(),mm->getFrameSet(),mm->getCameraParams());
    }

    double  evaluate_fitness(const std::vector<float>&sol ){
        aruco_mm::FullSceneOptimizer opt;
        aruco_mm::SparseLevMarq::eVector esol;esol.resize(sol.size());
        for(int i=0;i<sol.size();i++) esol(i)=sol[i];
        aruco_mm::SparseLevMarq::eVector error;

        opt.sba_frameSet=mm->getFrameSet();
        opt.sba_markerSet=mm->getMarkerSet();
        opt.error_optimization(esol,error);

        return error.cwiseProduct(error).sum();
    }

    void saveSolutionToPCD(string fp,const std::vector<float>&sol){
        aruco_mm::FullSceneOptimizer opt;

        aruco_mm::SparseLevMarq::eVector esol;esol.resize(sol.size());
        for(int i=0;i<sol.size();i++) esol(i)=sol[i];
        aruco_mm::MarkerSet ms;aruco_mm::FrameSet fs;

        fs=mm->getFrameSet();
        ms=mm->getMarkerSet();
        opt.fromVector(esol,ms,fs,mm->getCameraParams());
        aruco_mm::savePCDFile(fp,ms,fs);

    }

    std::vector<float> localsearch(const std::vector<float>&sol,int niters,bool verbose){
        aruco_mm::SparseLevMarq::eVector esol;esol.resize(sol.size());
        for(int i=0;i<sol.size();i++) esol(i)=sol[i];
        aruco_mm::FullSceneOptimizer opt;
         opt.sba_frameSet=mm->getFrameSet();
        opt.sba_markerSet=mm->getMarkerSet();
        aruco_mm::MarkerSet aux_ms=mm->getMarkerSet();
        aruco_mm::FrameSet aux_fs=mm->getFrameSet();
        auto cam_params=mm->getCameraParams();
        opt.fromVector(esol,aux_ms,aux_fs,cam_params);

        aruco_mm::FullSceneOptimizer::Params param;
        param.max_iters=niters;
        param.fixedMarkers={mm->getOriginMarkerId()};
        param.fix_camera_params=true;
        param.verbose=verbose;
        opt.optimize(aux_ms,aux_fs,cam_params,param);
        //now, get back the solution
        auto sol_opt=opt.toVector(aux_ms, aux_fs, cam_params);
        std::vector<float> vsol(sol.size());
        for(int i=0;i<sol.size();i++)vsol[i]=sol_opt(i);
        return vsol;
    }


    //rturns the number of variables of the solution vector that form part of the structure (markers)
    int getStructureSolSize()const
    {
        return  mm->getMarkerSet().size()*6;
    }
    //indicates how many variables comprise an structure element
    int getStrutureSize()const{return 6;}

};
////solves the sfm problem
class SfMAdapter:public Adapter{
public:
    std::vector<float>  getInitialSolution( ){
        aruco_mm::MarkerSetLagragianOptimizer opt;
        opt._fset=mm->getFrameSet();
        opt._mset=mm->getMarkerSet();
        auto vsol=opt.toVector( opt._mset, opt._fset,mm->getCameraParams());
        std::vector<float> vvsol(vsol.size());
        for(int i=0;i<vsol.size();i++) vvsol[i]=vsol(i);
        return vvsol;
    }
    double  evaluate_fitness(const std::vector<float>&sol ){
        aruco_mm::MarkerSetLagragianOptimizer opt;
        aruco_mm::SparseLevMarq::eVector esol;esol.resize(sol.size());
        for(int i=0;i<sol.size();i++) esol(i)=sol[i];
        aruco_mm::SparseLevMarq::eVector error;

        opt._fset=mm->getFrameSet();
        opt._mset=mm->getMarkerSet();
        opt._calc_restriction=false;
        opt.error_optimization(esol,error);
        double err=0;

        for(int i=0;i<error.size();i++) {
            float e=error(i);
            if (!isnan(e))
                err+= error(i)*error(i);
            else cerr<<"nan "<<i<<endl;
        }

        return err;
    }
    void saveSolutionToPCD(string fp,const std::vector<float>&sol){
        aruco_mm::MarkerSetLagragianOptimizer opt;

        aruco_mm::SparseLevMarq::eVector esol;esol.resize(sol.size());
        for(int i=0;i<sol.size();i++) esol(i)=sol[i];
        aruco_mm::MarkerSet ms;aruco_mm::FrameSet fs;

        fs=mm->getFrameSet();
        ms=mm->getMarkerSet();
        opt.fromVector(esol,ms,fs,mm->getCameraParams());
        aruco_mm::savePCDFile(fp,ms,fs);
    }

    std::vector<float> localsearch(const std::vector<float>&sol,int niters,bool verbose){
        aruco_mm::SparseLevMarq::eVector esol;esol.resize(sol.size());
        for(int i=0;i<sol.size();i++) esol(i)=sol[i];
        aruco_mm::MarkerSetLagragianOptimizer opt;
        cout<<"JHJ="<<mm->getFrameSet().size()<<endl;
        opt._fset=mm->getFrameSet();
        opt._mset=mm->getMarkerSet();
        aruco_mm::MarkerSet aux_ms=mm->getMarkerSet();
        aruco_mm::FrameSet aux_fs=mm->getFrameSet();
        auto cam_params=mm->getCameraParams();
        opt.fromVector(esol,aux_ms,aux_fs,cam_params);

        aruco_mm::MarkerSetLagragianOptimizer::Params param;
        param.max_iters=niters;
        param.fixedMarkers={mm->getOriginMarkerId()};
        param.fix_camera_params=true;
        param.verbose=verbose;
        param.applyRestrictions=false;
        opt.optimize(aux_ms,aux_fs,cam_params,param);
        //now, get back the solution
        auto sol_opt=opt.toVector(aux_ms, aux_fs, cam_params);
        std::vector<float> vsol(sol.size());
        for(int i=0;i<sol.size();i++)vsol[i]=sol_opt(i);
        return vsol;
    }


    int getStructureSolSize()const
    {
        return  mm->getMarkerSet().size()*12;
    }
    //indicates how many variables comprise an structure element
    int getStrutureSize()const{return 3;}

};

class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                                    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };


int main(int argc,char **argv){
try{
    CmdLineParser cml(argc,argv);

    if (argc<=2 || cml["-h"]){cerr<<"usage: in.mmp out.pcd [-t adapter_type:0 maker 1 sfm] [-ls <niters>:applies local search with the iterations indicated. Ouput pcd is saved to aux_ls.pcd]"<<endl;return -1;}
    std::shared_ptr<Adapter> sadap;
    if(stoi(cml("-t","0"))==0){
        sadap=std::make_shared<MarkerAdapter>();
        cerr<<"sss"<<endl;
    }
    else
        sadap=std::make_shared<SfMAdapter>();
     sadap->readFromFile(argv[1]);
     vector<float> sol=sadap->getInitialSolution();
     for(int i=0;i<10;i++)cout<<sol[i]<<endl;
     cout<<sol.size()<<endl;
    cerr<<"Error of solution: "<<sadap->evaluate_fitness(sol)<<endl;

    sadap->saveSolutionToPCD(argv[2],sol);
    if (cml["-ls"]){
        cerr<<"Running local search"<<endl;
        auto opt_ls_sol=sadap->localsearch(sol,std::stoi(cml("-ls")),true);
        cerr<<"Error of solution after local search: "<<sadap->evaluate_fitness(opt_ls_sol)<<endl;
        sadap->saveSolutionToPCD( "aux_ls.pcd",opt_ls_sol);
    }
    sadap->mm->saveToFile("aux.amm");
}catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}

