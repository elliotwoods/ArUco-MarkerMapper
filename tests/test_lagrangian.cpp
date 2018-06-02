#include <iostream>
#include "optimizers/sparselevmarq.h"
using namespace std;
double mu=1,lambda=1;



//auto fx=[](const sparselevmarq<double>::eVector &sol){return sol(0)*sol(0);};
//auto cx=[](const sparselevmarq<double>::eVector &sol){return sol(0)*sol(0)  -1;};

auto fx=[](const SparseLevMarq<double>::eVector &sol){return sol(0)*sol(0)+sol(1)*sol(1);};
auto cx=[](const SparseLevMarq<double>::eVector &sol){return sol(0)*sol(0) +sol(1)*sol(1) -1;};

void step(const SparseLevMarq<double>::eVector  &sol){
    lambda-=mu*cx(sol);
    mu*=1.5;
    cout<<sol<<endl;cin.ignore();

}

void err_xy_(const  SparseLevMarq<double>::eVector  &sol, SparseLevMarq<double>::eVector &err){
    err.resize(1);
    double constr=cx(sol);
   //err(0)= fx(sol) + mu/2. * constr*constr  - lambda*constr;
    err(0)= fx(sol) +  mu/2. * constr *constr - lambda*constr;
}

int main(){

        SparseLevMarq<double> solver;
        SparseLevMarq<double>::eVector sol(2);
        for(int i=0;i<sol.size();i++) sol(i)=0.2;
solver.verbose()=true;
solver.useLM()=true;
solver.allowErrorIncrement()=true;
        cout<<sol<<endl;
        solver.setStepCallBackFunc(std::bind(step,std::placeholders::_1));
        solver.solve(sol,std::bind(err_xy_,std::placeholders::_1,std::placeholders::_2));
        cout<<sol<<endl;


}
