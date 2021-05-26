#ifndef aruco_mm_SparseLevMarq_H
#define aruco_mm_SparseLevMarq_H
#include <Eigen/Sparse>
#include <functional>
#include <iostream>
#include <cmath>
#include "utils/mm_omp.h"
#include <ctime>
#include <cstring>
#include <vector>
#include <chrono>
#include <iomanip>
#include "marker_mapper_exports.h"
#include "debug.h"
namespace aruco_mm{
//Sparse Levenberg-Marquardt method for general problems
//Inspired in
//@MISC\{IMM2004-03215,
//    author       = "K. Madsen and H. B. Nielsen and O. Tingleff",
//    title        = "Methods for Non-Linear Least Squares Problems (2nd ed.)",
//    year         = "2004",
//    pages        = "60",
//    publisher    = "Informatics and Mathematical Modelling, Technical University of Denmark, {DTU}",
//    address      = "Richard Petersens Plads, Building 321, {DK-}2800 Kgs. Lyngby",
//    url          = "http://www.ltu.se/cms_fs/1.51590!/nonlinear_least_squares.pdf"
//}
template<typename T>
class   MARKERMAPPER_API SparseLevMarq{
public:


    typedef   Eigen::Matrix<T,Eigen::Dynamic,1>  eVector;
    typedef   std::function<void(const eVector  &, eVector &)> F_z_x;
    typedef   std::function<void(const eVector  &,  Eigen::SparseMatrix<T> &)> F_z_J;
    SparseLevMarq();
    /**
    * @brief Constructor with parms
    * @param maxIters maximum number of iterations of the algoritm
    * @param minError to stop the algorithm before reaching the max iterations
    * @param min_step_error_diff minimum error difference between two iterations. If below this level, then stop.
    * @param tau parameter indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
    * @param der_epsilon increment to calculate the derivate of the evaluation function
    * step will be very short. If near 0, means the opposite. This value is auto calculated in the subsequent iterations.
    */
    SparseLevMarq(int maxIters,T minError,T min_step_error_diff=0,T tau=1 ,T der_epsilon=1e-3);

 /**
 * @brief setParams
 * @param maxIters maximum number of iterations of the algoritm
 * @param minError to stop the algorithm before reaching the max iterations
 * @param min_step_error_diff minimum error difference between two iterations. If below this level, then stop.
 * @param tau parameter indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
 * @param der_epsilon increment to calculate the derivate of the evaluation function
 * step will be very short. If near 0, means the opposite. This value is auto calculated in the subsequent iterations.
 */
    void setParams(int maxIters,T minError,T min_step_error_diff=0,T tau=1 ,T der_epsilon=1e-3);

    /**
 * @brief solve  non linear minimization problem ||F(z)||, where F(z)=f(z) f(z)^t
 * @param z  function params 1xP to be estimated. input-output. Contains the result of the optimization
 * @param f_z_x evaluation function  f(z)=x
 *          first parameter : z :  input. Data is in T precision as a row vector (1xp)
 *          second parameter : x :  output. Data must be returned in T
 * @param f_J  computes the jacobian of f(z)
 *          first parameter : z :  input. Data is in T precision as a row vector (1xp)
 *          second parameter : J :  output. Data must be returned in T
 * @return final error
 */
    T solve(  eVector  &z, F_z_x , F_z_J);
/// Step by step solve mode


    /**
     * @brief init initializes the search engine
     * @param z
     */
    void init(eVector  &z, F_z_x );
    /**
     * @brief step gives a step of the search
     * @param f_z_x error evaluation function
     * @param f_z_J Jacobian function
     * @return error of current solution
     */
    bool step(  F_z_x f_z_x , F_z_J  f_z_J);
    bool step(  F_z_x f_z_x);
    /**
     * @brief getCurrentSolution returns the current solution
     * @param z output
     * @return error of the solution
     */
    T getCurrentSolution(eVector  &z);
    /**
     * @brief getBestSolution sets in z the best solution up to this moment
     * @param z output
     * @return  error of the solution
     */
    T getBestSolution(eVector  &z);

  /**  Automatic jacobian estimation
 * @brief solve  non linear minimization problem ||F(z)||, where F(z)=f(z) f(z)^t
 * @param z  function params 1xP to be estimated. input-output. Contains the result of the optimization
 * @param f_z_x evaluation function  f(z)=x
 *          first parameter : z :  input. Data is in T precision as a row vector (1xp)
 *          second parameter : x :  output. Data must be returned in T
 * @return final error
 */
    T solve(  eVector  &z, F_z_x );
    //to enable verbose mode
    bool & verbose(){return _verbose;}

//sets a callback func call at each step
    void setStepCallBackFunc(std::function<void(const eVector  &)> callback){_step_callback=callback;}
//sets a function that indicates when the algorithm must be stop. returns true if must stop and false otherwise
    void setStopFunction( std::function<bool(const eVector  &)> stop_function){_stopFunction=stop_function;}

    void  calcDerivates(const eVector & z ,  Eigen::SparseMatrix<T> &sJ,  F_z_x f_z_x);
private:
    int _maxIters;
    T _minErrorAllowed,_der_epsilon,_tau,_min_step_error_diff;
    bool _verbose;
    //--------
    eVector curr_z,x64;
    T currErr,prevErr,minErr ;
     Eigen::SparseMatrix<T> I,J;
    T mu,v;
    std::function<void(const eVector  &)> _step_callback;
    std::function<bool(const eVector  &)> _stopFunction;

    void  add_missing_diagonal_elements( Eigen::SparseMatrix<T> &M);
    void  get_diagonal_elements_refs_and_add( Eigen::SparseMatrix<T> &M,std::vector<T*> &d_refs,T add_val);
    void  mult(const Eigen::SparseMatrix<T> &lhs, const Eigen::SparseMatrix<T> &rhs,Eigen::SparseMatrix<T> &res);

};
template<typename T>
SparseLevMarq<T>::SparseLevMarq(){
    _maxIters=1000;_minErrorAllowed=0;_der_epsilon=1e-3;_verbose=false;_tau=1;v=5;_min_step_error_diff=0;
 }
/**
* @brief Constructor with parms
* @param maxIters maximum number of iterations of the algoritm
* @param minError to stop the algorithm before reaching the max iterations
* @param min_step_error_diff minimum error difference between two iterations. If below this level, then stop.
* @param tau parameter indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
* @param der_epsilon increment to calculate the derivate of the evaluation function
* step will be very short. If near 0, means the opposite. This value is auto calculated in the subsequent iterations.
*/
template<typename T>

SparseLevMarq<T>::SparseLevMarq(int maxIters,T minError,T min_step_error_diff,T tau ,T der_epsilon ){
    _maxIters=maxIters;_minErrorAllowed=minError;_der_epsilon=der_epsilon;_verbose=false;_tau=tau;v=5;_min_step_error_diff=min_step_error_diff;
 }

/**
* @brief setParams
* @param maxIters maximum number of iterations of the algoritm
* @param minError to stop the algorithm before reaching the max iterations
* @param min_step_error_diff minimum error difference between two iterations. If below this level, then stop.
* @param tau parameter indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
* @param der_epsilon increment to calculate the derivate of the evaluation function
* step will be very short. If near 0, means the opposite. This value is auto calculated in the subsequent iterations.
*/
template<typename T>
void SparseLevMarq<T>::setParams(int maxIters,T minError,T min_step_error_diff,T tau ,T der_epsilon){
    _maxIters=maxIters;
    _minErrorAllowed=minError;
    _der_epsilon=der_epsilon;
    _tau=tau;
    _min_step_error_diff=min_step_error_diff;

}


template<typename T>
void  SparseLevMarq<T>:: calcDerivates(const eVector & z ,  Eigen::SparseMatrix<T> &sJ,  F_z_x f_z_x)
{
std::vector< std::vector<Eigen::Triplet<T> > > sp_triplets(omp_get_max_threads());
#pragma omp parallel for
for (int i=0;i<z.rows();i++) {
     eVector zp(z),zm(z);
    zp(i)+=_der_epsilon;
    zm(i)-=_der_epsilon;
    eVector xp,xm;

    f_z_x( zp,xp);
    f_z_x( zm,xm);
    eVector dif=(xp-xm)/(2.f*_der_epsilon);

    //add the non zero elementos
    int tidx=omp_get_thread_num();
    for(int r=0;r<dif.rows();r++)
        if (fabs(dif(r))>1e-4)
            sp_triplets[tidx].push_back(Eigen::Triplet<T> (r,i,dif(r)));
}

//join all triplets
int n=0;
for(auto s:sp_triplets) n+=s.size();
std::vector<Eigen::Triplet<T> > sp_tripletsAll(n);
int cidx=0;
for(size_t i=0;i<sp_triplets.size();i++){
    memcpy(&sp_tripletsAll[cidx],& sp_triplets[i][0],sizeof(Eigen::Triplet<T>)*sp_triplets[i].size() );
    cidx+=sp_triplets[i].size() ;
}
sJ.setFromTriplets(sp_tripletsAll.begin(),sp_tripletsAll.end());
}


template<typename T>
T  SparseLevMarq<T>:: solve(  eVector  &z, F_z_x f_z_x){
return solve(z,f_z_x,std::bind(&SparseLevMarq<T>::calcDerivates,this,std::placeholders::_1,std::placeholders::_2,f_z_x));
}
template<typename T>
bool  SparseLevMarq<T>:: step(  F_z_x f_z_x){
return step(f_z_x,std::bind(&SparseLevMarq<T>::calcDerivates,this,std::placeholders::_1,std::placeholders::_2,f_z_x));
}

template<typename T>
void SparseLevMarq<T>::init(eVector  &z, F_z_x f_z_x ){
curr_z=z;
I.resize(z.rows(),z.rows());
I.setIdentity();
f_z_x(curr_z,x64);
// std::cerr<<x64.transpose()<<std::endl;
minErr=currErr=prevErr=x64.cwiseProduct(x64).sum();
 J.resize(x64.rows(),z.rows());
mu=-1;


}

template<typename T>
void SparseLevMarq<T>::get_diagonal_elements_refs_and_add( Eigen::SparseMatrix<T> &M,std::vector<T*> &refs,T add_val){
refs.resize(M.cols());
//now, get their references and add mu
for (int k=0; k<M.outerSize(); ++k)
for ( typename Eigen::SparseMatrix<T>::InnerIterator it(M,k); it; ++it)
    if (it.row()== it.col())     {refs[it.row()]= &it.valueRef(); *refs[it.row()]+=add_val;}
}


//parallel sparse matrix multiplication
//modyfied by rafael muñoz salinas (rmsalinas@uco.es) to make it parallel
template<typename T>
void  SparseLevMarq<T>::mult(const Eigen::SparseMatrix<T> &lhs, const Eigen::SparseMatrix<T> &rhs,Eigen::SparseMatrix<T> &res)
{

  // make sure to call innerSize/outerSize since we fake the storage order.
uint32_t rows = lhs.innerSize();
uint32_t cols = rhs.outerSize();
eigen_assert(lhs.outerSize() == rhs.innerSize());




typedef typename std::map<uint32_t,T> RowVal;
typedef typename std::pair<uint32_t,RowVal> Col_RowVal; //pair col-rowval
typedef typename std::vector< Col_RowVal> Col_RowValSet;



 std::vector<Col_RowValSet>omp_container(omp_get_max_threads());

// we compute each column of the result, in parallel after the other
#pragma omp parallel for
for (int32_t j=0; j<int32_t(cols); ++j)
{
    int tid=omp_get_thread_num();
    omp_container[tid].push_back( std::make_pair(j,RowVal()) );
    RowVal &row_val=omp_container[tid].back().second;
    for (typename Eigen::SparseMatrix<T>::InnerIterator rhsIt(rhs, j); rhsIt; ++rhsIt)
    {
        T y = rhsIt.value();
        uint32_t k = rhsIt.index();
        //add all indices
        for (typename Eigen::SparseMatrix<T>::InnerIterator lhsIt(lhs, k); lhsIt; ++lhsIt)
        {
            uint32_t i = lhsIt.index();
            T x = lhsIt.value();
            auto iter=row_val.find(i);
            if (iter==row_val.end()) row_val.insert(std::make_pair(i,x*y));
            else iter->second+=x*y;
        }
    }
}

//finally, unordered insertion
// unordered insertion
typedef  Eigen::SparseMatrix<T,Eigen::RowMajor,int32_t> RowMajorMatrix;
typedef  Eigen::SparseMatrix<T,Eigen::ColMajor,int32_t> ColMajorMatrix;
ColMajorMatrix resCol(rows,cols);

resCol.reserve( lhs.nonZeros() + rhs.nonZeros() );
for(auto & omp_thread:omp_container)
    for(auto c_r_v:omp_thread){//take each thread results
        int j=c_r_v.first;//column
        resCol.startVec(j);
        for(auto r_v:c_r_v.second)//for each column element, add it
            resCol.insertBackByOuterInnerUnordered(j,r_v.first) = r_v.second;
    }
resCol.finalize();
RowMajorMatrix resRow(resCol);
res = resRow;

}

template<typename T>
void SparseLevMarq<T>::add_missing_diagonal_elements(Eigen::SparseMatrix<T> &M){
  std::vector<bool> diag(M.rows(),false);
  for (int k=0; k<M.outerSize(); ++k)
     for (  typename Eigen::SparseMatrix<T>::InnerIterator it(M,k); it; ++it)
          if (it.row()== it.col())     diag[it.row()]=true;
  //and add them
  for(size_t i=0;i<diag.size();i++)   if (!diag[i]) M.insert(i,i) =0;

}
#define splm_get_time(a,b) std::chrono::duration_cast<std::chrono::duration<T>>(a-b).count()
template<typename T>
bool SparseLevMarq<T>::step( F_z_x f_z_x, F_z_J f_J){

auto t1= std::chrono::high_resolution_clock::now();
f_J(curr_z,J);
Eigen::SparseMatrix<T> Jt=J.transpose();
auto t2= std::chrono::high_resolution_clock::now();
 Eigen::SparseMatrix<T> JtJ;//=Jt*J;
 mult(Jt,J, JtJ);//parallel sparse matrix multiplication
auto t3= std::chrono::high_resolution_clock::now();
eVector  B=-Jt*x64;
auto t4= std::chrono::high_resolution_clock::now();
if(mu<0){//first time only
    T maxv=std::numeric_limits<T>::lowest();
    for (int k=0; k<JtJ.outerSize(); ++k)
        for (typename Eigen::SparseMatrix<T>::InnerIterator it(JtJ,k); it; ++it)
            if (it.row()== it.col())
                if (it.value()>maxv)
                    maxv=it.value();
    mu=maxv*_tau;
}


T gain=0,prev_mu=0;
std::vector<T*> refs;
int ntries=0;
bool isStepAccepted=false;
do{
    //add dumping factor to JtJ.
#if 1 //very efficient in any case, but particularly if initial dump does not produce improvement and must reenter
    if(refs.size()==0){//first time into the do
        add_missing_diagonal_elements(JtJ);
        get_diagonal_elements_refs_and_add(JtJ,refs,mu);
    }
    else for(size_t i=0;i<refs.size();i++)    *refs[i]+= mu-prev_mu;//update mu
    prev_mu=mu;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<T> > chol(JtJ);  // performs a Cholesky
#else  //less efficient, but easier to understand
    Eigen::SparseMatrix<T> A=JtJ+I*mu;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<T> > chol(A);  // performs a Cholesky
#endif

    eVector delta= chol.solve(B);
    eVector  estimated_z=curr_z+delta;
    //compute error
    f_z_x(estimated_z,x64);
    auto err=x64.cwiseProduct(x64).sum();
    auto L=0.5*delta.transpose()*((mu*delta) - B);
    gain= (err-prevErr)/ L(0,0) ;
     //get gain
    if (gain>0 && ((err-prevErr)<0)){
        mu=mu*std::max(T(0.33),T(1.-pow(2*gain-1,3)));
        v=2.f;
        currErr=err;
        curr_z=estimated_z;
        isStepAccepted=true;
    }
    else{ mu=mu*v; v=v*5;}

}while(gain<=0 && ntries++<5);
markermapper_debug_msg("Curr Error="<<currErr<<" AErr(prev-curr)="<<prevErr-currErr<<" gain="<<gain<<" dumping factor="<<mu,5);

if (_verbose) std::cout<<std::setprecision(5) <<"Curr Error="<<currErr<<" AErr(prev-curr)="<<prevErr-currErr<<" gain="<<gain<<" dumping factor="<<mu<<std::endl;
//    //check if we must move to the new position or exit
if ( currErr<prevErr)
std::swap ( currErr,prevErr );


if (0) {std::cerr<<" transpose="<<splm_get_time(t2,t1)<<" mult1="<< splm_get_time(t3,t2)<<" mult2="<< splm_get_time(t4,t3) <<std::endl;
          // std::cerr<<"solve="<<T(t4-t3)/T(CLOCKS_PER_SEC)<<std::endl;
}
return isStepAccepted;

}


template<typename T>
T  SparseLevMarq<T>:: getCurrentSolution(eVector  &z){

z=curr_z;
return currErr;
}
template<typename T>
T  SparseLevMarq<T>::solve( eVector  &z, F_z_x  f_z_x, F_z_J f_J){
prevErr=std::numeric_limits<T>::max();
init(z,f_z_x);

if( _stopFunction){
    do{
        step(f_z_x,f_J);
        if (_step_callback) _step_callback(curr_z);
    }while(!_stopFunction(curr_z));

}
else{
    //intial error estimation
    int mustExit=0;
    for ( int i = 0; i < _maxIters && !mustExit; i++ ) {
        if (_verbose)std::cerr<<"iteration "<<i<<"/"<<_maxIters<< "  ";
        bool isStepAccepted=step(f_z_x,f_J);
        //check if we must exit
        if ( currErr<_minErrorAllowed ) mustExit=1;
         if( fabs( prevErr -currErr)<=_min_step_error_diff  || !isStepAccepted) mustExit=2;
        //exit if error increment
        if (currErr<prevErr  )mustExit=3;
        //            if (  (prevErr-currErr)  < 1e-5 )  mustExit=true;
        if (_step_callback) _step_callback(curr_z);
    }

    std::cout<<"Exit code="<<mustExit<<std::endl;
}
z=curr_z;
return currErr;

}
}

#endif
