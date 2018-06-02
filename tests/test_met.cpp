#include "mappers/graph.h"
#include <iostream>
using namespace std;
using namespace aruco_mm;

double  get_expansiongraph_error(graph::Floyd   &falgo,graph::Graph<int> &cost_graph,int start_node){

    double err_sum=0;
    for(int i=0;i<cost_graph.size();i++){
        if (i!=start_node){
            auto path=falgo.path(start_node,i);
            assert(path.size()!=0);
            cout<<"-"<<start_node<<" ";
            for(int p=1;p<path.size();p++) {
                cout<<path[p]<< " ";
                assert(cost_graph.areConnected(path[p-1] , path[p]));
                err_sum+=cost_graph(path[p-1] , path[p]) ;
        }cout<<endl;
    }
    }

    return err_sum;
}


int main(){


    aruco_mm::graph::Graph<int> cost_graph(4,false);
    cost_graph.setDiagonalLinkValues(0);
    cost_graph(0,1)=cost_graph(1,0)=1.1;
    cost_graph(0,2)=cost_graph(2,0)=9.5;
    cost_graph(1,2)=cost_graph(2,1)=0.9;
    cost_graph(1,3)=cost_graph(3,1)=1.8;
    cost_graph(2,3)=cost_graph(3,2)=0.6;
graph::Floyd falgo;
falgo.process(cost_graph);
for(int i=0;i<4;i++)
    cout<<i<<":"<<  get_expansiongraph_error(falgo, cost_graph,i)<<endl;


}
