#include "posegraphoptimizer.h"
#include "graph.h"
#include <chrono>
#include "debug.h"
#include "optimizers/sparselevmarq.h"
#include "debug.h"

using namespace std;
namespace aruco_mm
{
/**
 */
uint32_t PoseGraphOptimizer::optimize(StgMatrix<Pose_Error> &pose_graph_io,bool removeOutlierLinks,const std::vector<std::string> &nodesInfo ){

    auto _to_string=[](int i){ 	std::stringstream str;str<<i;return str.str(); };

    if(nodesInfo.size()!=pose_graph_io.rows())
        nodes_info=nodesInfo;
    else{
        nodes_info.clear();
        for(size_t i=0;i< pose_graph_io.rows();i++)
            nodes_info.push_back(_to_string(i));
    }

    //reset information

    //obtains the initial pose graph
    std::map<uint32_t,uint32_t>   markers_id_pos;//given an id, returns its position in the poseGraph
    StgMatrix<se3> poseGraph;//pairwise poses
    graph::Graph<int> cost_graph;//cost between nodes in terms of reprojection error
    std::map<uint32_t,uint32_t>   markers_pos_id;//given a position in the posegraph, returns id



    auto time1=std::chrono::high_resolution_clock::now();
    //not read from file, do the real processing
    //compute_pairwise_information(_frameSet,_cam_params,poseGraph,cost_graph,markers_id_pos);
  //  for(auto mip:markers_id_pos) markers_pos_id.insert(make_pair(mip.second,mip.first));

    size_t nnodes=pose_graph_io.cols();
    assert(pose_graph_io.cols()==pose_graph_io.rows());
    cost_graph.setParams(nnodes,true);
    poseGraph.resize(nnodes,nnodes);
    for(size_t i=0;i<nnodes;i++){
        cost_graph(i)=i;
        for(size_t j=0;j<nnodes;j++){
            if (i==j){
                cost_graph(i,i)=cost_graph.getInfinity();
            }
            else{
                poseGraph(i,j) =pose_graph_io(i,j).pose;
                if (!poseGraph(i,j).isValid()) cost_graph(i,j)=cost_graph.getInfinity();
                else cost_graph(i,j)=pose_graph_io(i,j).repj_error;
            }
        }
    }
//    for(size_t i=0;i<nnodes;i++){
//         for(size_t j=0;j<nnodes;j++)
//             if ( cost_graph(i,j)==cost_graph.getInfinity()) cout<<"inf\t";
//         else
//               cout<<cost_graph(i,j)  <<"\t";
//         cout<<endl;
//    }



    //Compute paths between all elements

    graph::Floyd falgo;
    falgo.process(cost_graph);
    //find graph components. Hopefully, there is only one (ie,  there is a path between each pair of nodes)
    auto v_gg=getGraphComponents(cost_graph,poseGraph,falgo);
    //more than one?
    if (v_gg.size()!=1){
        cerr<<"IMPORTANT. THERE ARE MORE THAN 1 COMPONENTS IN THE GRAPH. USING ONLY THE BIGGEST ONE"<<endl;
        cost_graph=v_gg[0].first;
        poseGraph=v_gg[0].second;
        falgo.process(cost_graph);//recompute the floyd
    }

    //find the best path to all the other nodes. Ie, select the starting node from wich start expasion
    double minErr=std::numeric_limits<double>::max();int best_org=-1;
    for(int i=0;i<cost_graph.size();i++){
        double err=get_expansiongraph_error(falgo,cost_graph,i);
        if (err<minErr){minErr=err;best_org=i; }
    }

    //now that the minimum expansion tree is selected, save it and recompute floyd
    cout<<"minERR "<<minErr<<" best="<<best_org<<endl;
 //   uint32_t origin_id=markers_pos_id[best_org];
 //   markermapper_debug_msg("starting from "<<origin_id<<" with error="<<minErr,5);
    auto best_exp_graph=get_expansion_graph( cost_graph,falgo,best_org);
    falgo.process(best_exp_graph);


    //remove links with too much error to avoid using them
    if (removeOutlierLinks){
        int nremoved=removeOutlierLinksFromCostGraph(best_exp_graph,cost_graph,3);
        if (nremoved!=0){
            best_exp_graph=get_expansion_graph( cost_graph,falgo,best_org);
            falgo.process(best_exp_graph);
            if ( getGraphComponents(cost_graph,poseGraph,falgo).size()!=1)throw std::runtime_error("It should not have happened");
        }
    }

//    for(size_t i=0;i<nnodes;i++){
//         for(size_t j=0;j<nnodes;j++)
//             if ( cost_graph(i,j)==cost_graph.getInfinity()) cout<<"inf\t";
//         else
//               cout<<cost_graph(i,j)  <<"\t";
//         cout<<endl;
//    }

    auto time2=std::chrono::high_resolution_clock::now();
    cout<<"PosegraphCreation="<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(time2-time1).count())/1e9<<"s"<<endl;

    //now, global optimization using the cycles
    vector<vector<uint32_t> > all_cycles=basic_cycles(cost_graph,best_exp_graph);


    //now, expand creating a posed
    poseGraph = rotational_translational_error_minimization( all_cycles ,poseGraph,cost_graph);


    pose_graph_io.clear();
    pose_graph_io.resize(nnodes,nnodes);

    for(size_t i=0;i<nnodes;i++){
        for(size_t j=0;j<nnodes;j++){
             pose_graph_io(i,j).pose =poseGraph(i,j);
            pose_graph_io(i,j).repj_error=cost_graph(i,j);
        }
    }

    return best_org;

}



//finds the different components of a graph
vector<std::pair< graph::Graph<int>,StgMatrix<se3> > >  PoseGraphOptimizer::getGraphComponents(graph::Graph<int> &cost_graph,StgMatrix<se3> &poseGraph,const graph::Floyd &floyd){

    //extracts the nodes from a graph and creates new graphs
    auto extractNodes=[](graph::Graph<int> &cost_graph,StgMatrix<se3> &poseGraph,const vector<uint32_t>  &nodes ){
        std::map<uint32_t,uint32_t> old_new_indices;
        int nidx=0;
        for(auto n:nodes) old_new_indices.insert(make_pair(n,nidx++));
        std::pair< graph::Graph<int>,StgMatrix<se3> > res;
        res.first.setParams(nodes.size(),false);
        res.second.resize(nodes.size(),nodes.size());
        for(auto ii:old_new_indices){
            res.first(ii.second)=cost_graph(ii.first);
            for(auto ii2:old_new_indices){
                res.first(ii.second,ii2.second) = cost_graph(ii.first,ii2.first);
                res.second(ii.second,ii2.second)= poseGraph(ii.first,ii2.first);
            }
        }
        return res;
    };


    std::map<uint32_t,uint32_t>  node_comp;//given a  node, returns its component

    graph::Floyd falg=floyd;
    if (!floyd.isValid()) falg.process(cost_graph);
    //find elements a node is connected to

    int ncomponents=0;
    for(int i=0;i<cost_graph.size();i++){
        //if node has not a component already, create the component
        if ( node_comp.find(i)==node_comp.end()) node_comp[i]=ncomponents++;
        for(int j=i+1;j<cost_graph.size();j++){
            if (falg.areConnected(i,j)){
                node_comp[j]=node_comp[i];
            }
        }
    }


    std::map<uint32_t,vector<uint32_t> >  comp_nodes;
    for(auto nc:node_comp)
        comp_nodes[nc.second].push_back(nc.first);

    cout<<"there are "<<comp_nodes.size()<<" components"<<endl;
    for(auto com:comp_nodes){
        cout<<"component:"<<com.first<<":";
        for(auto n:com.second) cout<<nodes_info[n]<<" "; //cout<<cost_graph(n)<<" ";
        cout<<endl;
    }

    vector<std::pair< graph::Graph<int>,StgMatrix<se3> > >  components;

    for(auto cn:comp_nodes)
        components.push_back( extractNodes(cost_graph,poseGraph, cn.second));

    //SORT BY NUMBER OF COMPONENTS
    std::sort(components.begin(),components.end(),[](const std::pair< graph::Graph<int>,StgMatrix<se3> > &a,const std::pair< graph::Graph<int>,StgMatrix<se3> > &b){return a.first.size()>b.first.size();});
    return components;
}

//AQUI!!!
double PoseGraphOptimizer::get_expansiongraph_error(graph::Floyd   &falgo,graph::Graph<int> &cost_graph,int start_node){

    double err_sum=0;
    for(int i=0;i<cost_graph.size();i++){
        if (i!=start_node){
            auto path=falgo.path(start_node,i);
            //  cout<<start_node<<" "<<i<<endl;
            assert(path.size()!=0);
            for(size_t p=1;p<path.size();p++) {
                assert(cost_graph.areConnected(path[p-1] , path[p]));
                err_sum+=cost_graph(path[p-1] , path[p]) ;
            }
        }
    }

    return err_sum;
}

graph::Graph<int> PoseGraphOptimizer::get_expansion_graph(graph::Graph<int> &cost_graph, graph::Floyd &falgo,int start_node){
     graph::Graph<int> exp_graph(cost_graph.size(),false);
    exp_graph.setDiagonalLinkValues(0);
    for(int i=0;i<cost_graph.size();i++){
        exp_graph(i)=cost_graph(i);
        if (i!=start_node){
            auto path=falgo.path(start_node,i);
            for(size_t p=1;p<path.size();p++) {
                exp_graph(path[p-1],path[p])=cost_graph(path[p-1] , path[p]);
                exp_graph(path[p],path[p-1])=cost_graph( path[p],path[p-1]);
            }
        }
    }
     return exp_graph;
}





int PoseGraphOptimizer::removeOutlierLinksFromCostGraph(graph::Graph<int> &exp_graph,graph::Graph<int> &cost_graph,double ndevs){

    auto getNofComponents=[](graph::Graph<int> &graph){
        std::map<uint32_t,uint32_t>  node_comp;//given a  node, returns its component

        graph::Floyd falg;
        falg.process(graph);
        //find elements a node is connected to

        int ncomponents=0;
        for(int i=0;i<graph.size();i++){
            //if node has not a component already, create the component
            if ( node_comp.find(i)==node_comp.end()) node_comp[i]=ncomponents++;
            for(int j=i+1;j<graph.size();j++){
                if (falg.areConnected(i,j)){
                    node_comp[j]=node_comp[i];
                }
            }
        }
        return ncomponents;
    };

    //now, remove outliers from the cost_graph to avopid using these links in the cycles
     double sum=0,sum_sq=0,n=0;
    for(int i=0;i<exp_graph.size();i++)
        for(int j=i+1;j<exp_graph.size();j++)
                if (exp_graph.areConnected(i,j)) {
                    sum+=  exp_graph(i,j);
                    sum_sq+=exp_graph(i,j)*exp_graph(i,j);
                    n++;
                }
    //now, compute mean and dev
    double mean=sum/n;
    double dev= sqrt(sum_sq/n - mean*mean);
    //now, annotate elements to remove
    struct rem{
        int i,j;
        double cost;
    };
    vector<rem> toRemove;
    for(int i=0;i<cost_graph.size();i++)
        for(int j=i+1;j<cost_graph.size();j++)
                if (!exp_graph.areConnected(i,j) && cost_graph.areConnected(i,j)) {
                    if (fabs(cost_graph(i,j)-mean)>ndevs*dev  ) {
                        toRemove.push_back({i,j,cost_graph(i,j)});
                        cost_graph(i,j)=cost_graph(j,i)=cost_graph. getInfinity();
                    }
                }
    //sort by decreaseing cost
    std::sort(toRemove.begin(),toRemove.end(),[](const rem&a,const rem&b){ return a.cost>b.cost;});

    //remove one by one, checking that the graph can keep the connection between the elements even
    int nRemoved=0;
    for(auto r:toRemove){
        cout<<"Outlier link "<<r.i<<" "<<r.j<<" "<<r.cost <<endl;
        float cost=        cost_graph(r.i,r.j);
        cost_graph(r.i,r.j)=cost_graph(r.j,r.i)=cost_graph. getInfinity();
        if ( getNofComponents(cost_graph)>1){
            cout<<"attempt to remove link "<<r.i<<" "<<r.j<<" disconnects the graph into ore than one components. Not removed. But there might an oulier connection"<<endl;
            cost_graph(r.i,r.j)=cost_graph(r.j,r.i)=cost;
        }
        else nRemoved++;


    }
return nRemoved;
}

std::vector< std::vector<uint32_t> > PoseGraphOptimizer::basic_cycles( graph::Graph<int> &cost_graph,graph::Graph<int> &expansion_graph){
    graph::Floyd  FloydExpansionGraph;
    FloydExpansionGraph.process(expansion_graph);
    std::vector< std::vector<uint32_t> >cycles;

    for(int i=0;i<expansion_graph.size();i++){
        for(int j=i+1;j<expansion_graph.size();j++){
            if ( !expansion_graph.areConnected(i,j) &&  cost_graph.areConnected(i,j)){
                //find the path and add the connection
                cycles.push_back(FloydExpansionGraph.path(i,j));
            }
        }
    }
    return cycles;
}



StgMatrix<  se3  > PoseGraphOptimizer::rotational_translational_error_minimization( const vector<vector<uint32_t> > &basic_cycles, StgMatrix<  se3 >   &_mean_graph, graph::Graph<int> &cost_graph){



    vector<vector<double> > basic_cycles_weights(basic_cycles.size());
    //compte the weight of each egde in a cycle to give more confidence to  edges with low repj error
    for(size_t bc=0;bc<basic_cycles.size();bc++){
        const auto &cycle=basic_cycles[bc];
        //now, give assign a weight
        basic_cycles_weights[bc].resize(cycle.size());
        auto &cycle_weigths=basic_cycles_weights[bc];
        double sum=0;
        for(size_t i=0;i<cycle_weigths.size();i++)
                sum+=cycle_weigths[i]=1./(cost_graph(  cycle[i], cacc(cycle,i+1) )+0.1);
        //now, normalize
        for(size_t i=0;i<cycle_weigths.size();i++)
            cycle_weigths[i]/=sum;
    }

    //return rotational_error_minimization(basic_cycles,_mean_graph,basic_cycles_weights);
    auto rotate_corrected=rotational_error_minimization(basic_cycles,_mean_graph,basic_cycles_weights);
    return translational_error_minimization(basic_cycles,rotate_corrected,basic_cycles_weights);
}


StgMatrix<  se3  > PoseGraphOptimizer::rotational_error_minimization( const vector<vector<uint32_t> > &basic_cycles, StgMatrix<  se3 >   &initial_posegraph, const vector<vector<double> > &basic_cycles_weight){

    auto getRotationalError=[]( const vector<vector<uint32_t> > &basic_cycles,   StgMatrix<se3> &_mean_graph){
        //total rotational error
        double err=0;

        {
            for(auto cycle: basic_cycles){
                for(size_t s=0;s<cycle.size();s++){
                    cv::Mat r=cv::Mat::eye(3,3,CV_32F),r33;
                    for(size_t i=0;i<cycle.size();i++){
                        assert ( !isnan( _mean_graph (cycle[ (s+i) %cycle.size()] , cycle[(s+i+1) % cycle.size()] ).getRotation3x3().at<float>(0,0)) );
                        r= _mean_graph( cycle[ (s+i) %cycle.size()] , cycle[(s+i+1) % cycle.size()] ).getRotation3x3()*r;
                    }
                    err+= cv::norm( r-cv::Mat::eye(3,3,CV_32F));
                }
            }
            //cout<<"EER2="  <<err<<endl;
        }


        return err;

    };

    auto avrg_Mat=[](const vector<cv::Mat> &vec){cv::Mat res=cv::Mat::zeros(vec[0].size(),vec[0].type());for(auto v:vec) res+=v;res*=1./double(vec.size());return res;};

     StgMatrix<  se3  >  curr_posegraph=initial_posegraph;
            //take a cycle and test the rotation correction

    //here, we keep the best solution
    double best_posegraph_err=getRotationalError(basic_cycles,curr_posegraph);
    StgMatrix<  se3  >  best_posegraph=curr_posegraph;

    ofstream outplotFile;
    if(debug::Debug::getLevel()==1111)  outplotFile.open("rot.txt");

    //copy only data in our cycles


    //copy data here

    int maxIters=300;
    double minErr=1e-3;
    int it=0;
    double cur_r_err=getRotationalError(basic_cycles,curr_posegraph);
    markermapper_debug_msg("R error="<<cur_r_err<<" "<<minErr,5);
    if(debug::Debug::getLevel()==1111) outplotFile<<it<<" "<<cur_r_err<<endl;
    double prev_r_err=std::numeric_limits<double>::max();
    cv::Mat r=cv::Mat::eye(3,3,CV_32F),aux1(3,3,CV_32F),aux2(3,3,CV_32F),rvec(1,3,CV_32F),sol(3,3,CV_32F);
    do{
        StgMatrix<  vector<cv::Mat>  > newRotations( initial_posegraph.rows(),initial_posegraph.cols());
        // StgMatrix< vector<double> > translation_error( _mean_graph.rows(),_mean_graph.cols());
        for(size_t cc=0;cc< basic_cycles.size();cc++){
            auto cycle=basic_cycles[cc];
            //go for translation, which is easier

            for(size_t s=0;s<cycle.size();s++){
                setEye(r);
                //traverse and compute rotation
                for(size_t i=0;i<cycle.size();i++){
                    assert ( !isnan( curr_posegraph (  cycle[ (s+i) %cycle.size()] , cycle[(s+i+1) % cycle.size()] ).getRotation3x3().at<float>(0,0)) );

                     curr_posegraph (  cycle[ (s+i) %cycle.size()] , cycle[(s+i+1) % cycle.size()] ).getRotation3x3(aux1);
                     m33mult( aux1,r,aux2);
                     memcpy(r.ptr<float>(0),aux2.ptr<float>(0),9*sizeof(float));

                 }
                //find the matrix that makes identity


                inverseR(r);
                rodrigues_M2V(r,rvec);

                //use only a fraction of the rotation
                float *rptr=rvec.ptr<float>(0);
                float f=basic_cycles_weight[cc][s];
                rptr[0]*=f;     rptr[1]*=f;rptr[2]*=f;
                //back to matrix
                rodrigues_V2M(rvec,r);

                //applied to the first one is the new location
                curr_posegraph(  cycle[ s %cycle.size()]  , cycle[(s+1) % cycle.size() ]).getRotation3x3(aux1);
                m33mult( aux1,r,sol);

                //back to rvec
                rodrigues_M2V(sol,rvec);
                //save
                newRotations( cycle[ s ]  , cycle[ (s+1)   %cycle.size() ] ) .push_back(rvec.clone());
                //inverse
                inverseR(sol);
                rodrigues_M2V(sol,rvec);
                newRotations( cycle[ (s+1)   %cycle.size() ]  , cycle[ s ]  ).push_back(rvec.clone());
            }
        }
        //average results
        for(size_t i=0;i<newRotations.rows();i++)
            for(size_t j=0;j<newRotations.cols();j++){
                if (newRotations(i,j).size()!=0 )
                    curr_posegraph(i,j).setRotation( avrg_Mat( newRotations(i,j)) );
            }

        swap(cur_r_err,prev_r_err);
        cur_r_err=getRotationalError(basic_cycles,curr_posegraph);

        markermapper_debug_msg("R error="<<cur_r_err<<" iteration "<<it<<"/"<<maxIters,5);
        if(debug::Debug::getLevel()==1111) outplotFile<<it<<" "<<cur_r_err<<endl;

        //          cout<<"T error= "<<getTranslationalError(basic_cycles,corrected_mean_graph)<<endl;

        if (cur_r_err<best_posegraph_err){
            best_posegraph_err=cur_r_err;
            best_posegraph=curr_posegraph;
        }

    }    while(it++<maxIters  && cur_r_err>minErr && cur_r_err<prev_r_err);

    markermapper_debug_msg("Final rotational error="<<best_posegraph_err,5);

    //find decoupling translation
    StgMatrix<  vector<cv::Mat>  > corr_t(best_posegraph.rows(),best_posegraph.cols());
    for(size_t cc=0;cc< basic_cycles.size();cc++){
        auto cycle=basic_cycles[cc];
        for(size_t s=0;s<cycle.size();s++){
            //get the central point between elements

            int i=cycle[s];
            int j=cycle[(s+1)%cycle.size()];
            for(int n=0;n<2;n++){

                auto c2=initial_posegraph(j,i).getTvec()*0.5;
                auto t_decoupled= initial_posegraph(i,j).getRotation3x3()*c2.t()-best_posegraph(i,j).getRotation3x3()*c2.t() + best_posegraph(i,j).getTvec().t();

                //                   cout<<"dif=="<<t_decoupled.t()- corrected_mean_graph(i,j).getTvec()<<endl;
                corr_t(i,j).push_back(t_decoupled);
                swap(i,j);//do the opposite too
            }

        }
    }
    //now, move data to corrected graph
    for(size_t i=0;i<corr_t.rows();i++)
        for(size_t j=0;j<corr_t.cols();j++)
        {
            if (corr_t(i,j).size()!=0)
                best_posegraph(i,j).setTranslation( avrg_Mat( corr_t(i,j)));
        }
    return best_posegraph;

}

double PoseGraphOptimizer::getTranslationalError( const vector<vector<uint32_t> > &basic_cycles,   StgMatrix<  se3 >   &_mean_graph){
    //total rotational error

    double err=0;
    for(auto cycle: basic_cycles){
        cv::Mat rt=cv::Mat::eye(4,4,CV_32F);
        for(size_t s=0;s<cycle.size();s++){
            int a=cycle[ s ],b=cycle[ (s+1) % cycle.size() ];
            rt=_mean_graph(a,b).convert()*rt;
        }
        err+= se3::convert(rt).tnorm();
    }

    return err;

}
StgMatrix<  se3  > PoseGraphOptimizer::translational_error_minimization( const vector<vector<uint32_t> > &basic_cycles, StgMatrix<  se3 >   &_graph,const vector<vector<double> > &basic_cycles_weights){
(void)basic_cycles_weights;
    std::set<uint32_t> Edges;//set of edges in the cycles
    for(auto &cycle:basic_cycles)
        for(size_t s=0;s<cycle.size();s++)
            Edges.insert(  toEdge(cacc(cycle,s),cacc(cycle,s+1)));

    //move data, but avoid the duplication due to simetry
    std::map<uint32_t,uint32_t> edge_pos;//for each edge, its position in the solution vector
    StgMatrix<  se3  > tcorrected_graph(_graph.rows(),_graph.cols());
    for(auto e:Edges){
        auto i_j=fromEdge(e);
        edge_pos.insert(make_pair(e,edge_pos.size()));
        //    cout<<e<<" "<<i_j.first<<" "<<i_j.second<<endl;
        tcorrected_graph(i_j.first,i_j.second)=_graph(i_j.first,i_j.second);
    }

    //save info in the edges

    SparseLevMarq<double>::eVector sol(Edges.size()*3);
    { int sol_idx=0;
        for(auto e:Edges){
            auto a_b=fromEdge(e);
            //check which combination is stored and save it
            se3 rt=tcorrected_graph( a_b.first,a_b.second);
            for(int i=0;i<3;i++) sol(sol_idx++)=rt.rt[i+3];
        }
    }
    auto initial_solution=sol;
    auto cycle_error=[&](const  SparseLevMarq<double>::eVector  &c_sol, vector<uint32_t> cycle){
        //apply the cycle and get the error using this values
        cv::Mat RT=cv::Mat::eye(4,4,CV_32F);
        cv::Mat rtcv,aux(4,4,CV_32F);
//        for(size_t s=0;s<cycle.size();s++){
//            auto e=toEdge(cycle[s],cacc(cycle,s+1));
//            //get its position in the solution vector
//            int off=edge_pos[e]*3;
//            auto i_j=fromEdge(e);
//            //change

//            se3 rt=tcorrected_graph(i_j.first,i_j.second );
//            //change t
//            for(int i=0;i<3;i++) rt.rt[3+i]= c_sol( off++);
//            //is inverted?

//            if ( i_j.first!=cycle[s]) rt=rt.inverse();
//            RT=  rt.convert()*RT;
//        }

        for(size_t s=0;s<cycle.size();s++){
            auto e=toEdge(cycle[s],cacc(cycle,s+1));
            //get its position in the solution vector
            int off=edge_pos[e]*3;
            auto i_j=fromEdge(e);
            //change

            se3 rt=tcorrected_graph(i_j.first,i_j.second );
            //change t
            for(int i=0;i<3;i++) rt.rt[3+i]= c_sol( off++);
            rodrigues_RTVec2M44(rt.rt,rtcv);
            //is inverted?

            if ( i_j.first!=cycle[s]){
                inverseRT(rtcv,aux);
                matmul(aux.ptr<float>(0),RT.ptr<float>(0),rtcv.ptr<float>(0));
                memcpy(RT.ptr<float>(0),rtcv.ptr<float>(0),16*sizeof(float));
            }
            else{
                matmul(rtcv.ptr<float>(0),RT.ptr<float>(0),aux.ptr<float>(0));
                memcpy(RT.ptr<float>(0),aux.ptr<float>(0),16*sizeof(float));
            }

//            RT=  rt.convert()*RT;
        }


        assert(RT.type()==CV_32F);
        //the translation vector is the error
        return cv::Vec3f (RT.at<float>(0,3),RT.at<float>(1,3),RT.at<float>(2,3));
    };

    auto err_func=[&]( const  SparseLevMarq<double>::eVector  &sol, SparseLevMarq<double>::eVector &err){
        int add_langrian=1;
        err.resize(3*basic_cycles.size()+ add_langrian*initial_solution.size());
        int err_idx=0;
        for(size_t i=0;i<basic_cycles.size();i++){
            auto e=cycle_error(sol,basic_cycles[i]);
            for(int j=0;j<3;j++) err(err_idx++)=e[j];
        }
        //now, add also the distance to the original solution as a constrain
        if (add_langrian)
            for(int i=0;i<initial_solution.size();i++)
                err(err_idx++)= initial_solution(i)-sol(i);
    };

    SparseLevMarq<double>::eVector err  ;
    err_func(sol,err);
    SparseLevMarq<double> Solver;
    cout<<"min Err="<<1e-3*basic_cycles.size()<<endl;
    Solver.setParams(100,1e-3*basic_cycles.size(),1e-4);
    Solver.verbose()=true;
    markermapper_debug_msg("  TRANSLATIONAL ERROR MINIMIZATION",5);

    Solver.solve(sol,err_func);
    err_func(sol,err);

    auto resulting_graph=_graph;
    //now, transfer data to resulting graph
    {
        int sol_idx=0;
        for(auto e:Edges){
            auto a_b=fromEdge(e);
            //check which combination is stored and save it
            se3 &rt=resulting_graph( a_b.first,a_b.second );
            for(int i=0;i<3;i++) rt.rt[i+3]=sol(sol_idx++);
        }
    }

    //create the symetric elements
    for(size_t i=0;i<tcorrected_graph.rows();i++)
        for(size_t j=0;j<tcorrected_graph.cols();j++)
            if (tcorrected_graph( i, j  ).isValid()  && !tcorrected_graph( j,i  ).isValid()  ) resulting_graph( j,i  )=resulting_graph( i, j  ).inverse();
    markermapper_debug_msg("T init= "<<getTranslationalError(basic_cycles,_graph),5);
    markermapper_debug_msg("T error= "<<getTranslationalError(basic_cycles,resulting_graph),5);
    return resulting_graph;
}



}
