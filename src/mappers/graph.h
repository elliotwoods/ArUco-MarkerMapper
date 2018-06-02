/**
 *       @file  graph.h
 *     @author  Rafael Muñoz Salinas (), rmsalinas@uco.es
 *
 *   @internal
 *     Created  17/09/10
 *    Revision 01/10/11 - 13:51:58
 *    Compiler  gcc/g++
 *     Company  University of Cordoba, Spain
 *   Copyright  Copyright (c) 2010, University of Cordoba, Spain
 *
 * This source code is released for free distribution under the terms of the
 * GNU General Public License as published by the Free Software Foundation.
 * =======================================================================
 */

#ifndef __GUIA_GRAPH_H__
#define __GUIA_GRAPH_H__
#include <cassert>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <limits>
namespace aruco_mm
{
namespace graph
{

/** \brief This class represents a graph */
template<class NodeType>
class Graph
{
public:
    /** @brief Empty Constructor.*/
    Graph() :   _nNodes(0)
    { }

    Graph(unsigned int n,bool directed):    _nNodes(0){
        setParams(n,directed);
    }

    Graph(unsigned int n,bool directed,double value):   _nNodes(0){
        setParams(n,directed);
        setTo(value);
    }


    /** @brief Destructor. */
    virtual ~Graph() {}

    /** @brief Gives size to the graph. */
    void setParams(unsigned int n,bool directed);

    /** @brief Indicates the number of nodes. */
    int size( void ) const
    {
        return _nNodes;
    }

    /** @brief Indicates whether the graph is directed.  */
    bool isDirected() const
    {
        return _isDirected;
    }

    /** @brief makes the graph directed or indirected */
    void setDirected(bool v)
    {
        _isDirected=v;
    }

    /** @brief Gets the weight between to nodes */
    double & operator()( unsigned int i, unsigned int j )
    {
        assert(i<(unsigned int)(_nNodes) && j<(unsigned int)(_nNodes));
        return _links[i][j];
    }
    double   operator()( unsigned int i, unsigned int j )const
    {
        assert(i<(unsigned int)(_nNodes) && j<(unsigned int)(_nNodes));
        return _links[i][j];
    }
    //sets all links to value indicated
    void setTo(double val){
        for(int i=0;i<_nNodes;i++)
            for(int j=0;j<_nNodes;j++) _links[i][j]=val;

    }


    /**Indicates if nodes are connected */
    bool areConnected(unsigned int i,unsigned int j)const{
        return _links[i][j]!= getInfinity() || _links[j][i]!= getInfinity();
    }

    /** @brief Gets the node indicated */
    NodeType & operator()( unsigned int i  )
    {
        assert(i< (unsigned int)(_nNodes) );
        return _nodes[i];
    }

    void setDiagonalLinkValues(double v); /**Set diagonal values */

    /** @brief Returns an string showing the internal values */
    std::string toString() const;

    /** @brief Saves this object to a file */
    void saveToFile(std::string filePath) const ;

    /** @brief Reads this object from a file */
    void readFromFile(std::string filePath) ;

     double getInfinity()const{return std::numeric_limits<double>::max();} /**The value of an unexsisting link */


    void toStream(std::ostream &str);
    void fromStream(std::istream &str);
protected:

    // N�mero de nodos del grafo.
    int _nNodes;
    bool _isDirected;
    // Matriz de conexiones del grafo.
    std::vector<std::vector<double > > _links;
    std::vector<NodeType > _nodes;
};




//////////////////////////////////
//
//
//////////////////////////////////
template<class NodeType>
void Graph<NodeType>::setParams(unsigned int n,bool directed)
{
    _nNodes=n;
    _isDirected=directed;
    _links.resize(_nNodes);
    _nodes.resize(_nNodes);
    for (int i=0; i<_nNodes; i++)
    {
        _links[i].resize(_nNodes);
        for (int j=0; j<_nNodes; j++) _links[i][j]= getInfinity();
    }
}

//////////////////////////////////
//
//
//////////////////////////////////
template<class NodeType>
void Graph<NodeType>::setDiagonalLinkValues(double v)
{
  assert(_nNodes!=0);
  for (int i=0; i<_nNodes; i++) _links[i][i]=v;
}

//////////////////////////////////
//
//
//////////////////////////////////
template<class NodeType>
std::string Graph<NodeType>::toString() const
{
  std::stringstream ret;
  ret<<"Links\n";
  for (int i=0; i<_nNodes; i++)
  {
    for (int j=0; j<_nNodes; j++)
        if( _links[i][j]==Graph:: getInfinity())
            ret<<"  -  ";
      else ret<<_links[i][j]<<" ";
    ret<<"\n";
  }
  ret<<"Nodes\n";
  for (int i=0; i<_nNodes; i++)
    ret<<_nodes[i]<<" ";
  ret<<std::endl;
  return ret.str();
}

//////////////////////////////////
//
//
//////////////////////////////////
template<class NodeType>
void Graph<NodeType>::saveToFile(std::string filePath) const
{
  std::ofstream file(filePath.c_str());
  if (!file) throw std::runtime_error("Graph<NodeType>::saveToFile file could not be opened for writting:"+filePath);
  int magic=14198031;
  file.write((char*)&magic,sizeof(int));
  toStream(file);
}


//////////////////////////////////
//
//
//////////////////////////////////
template<class NodeType>
void Graph<NodeType>::readFromFile(std::string filePath)
{
  std::ifstream file(filePath.c_str());
  if (!file)
    throw std::runtime_error("Graph<NodeType>::saveToFile file could not be opened for reading:"+filePath);
  int magic=14198031;
  file.read((char*)&magic,sizeof(int));
  if (magic!=14198031)
    throw std::runtime_error("Graph<NodeType>::saveToFile invalid file type:"+filePath);
  fromStream(file);
}
template<class NodeType>
void Graph<NodeType>:: toStream(std::ostream &file) {
    file.write((char*)&_nNodes,sizeof(int));
    file.write((char*)&_isDirected,sizeof(bool));
    for ( int i=0; i<_nNodes; i++)
        file.write(reinterpret_cast<const char *>(&_links[i][0]),sizeof(_links[i][0])*_nNodes);
    file.write( (char*)(&_nodes[0]),_nodes.size()*sizeof(_nodes[0]));
}

template<class NodeType>
void Graph<NodeType>:: fromStream(std::istream &file){

    int nNodes,isDirected;
    file.read((char*)&nNodes,sizeof(int));
    file.read((char*)&isDirected,sizeof(bool));
    setParams(nNodes,isDirected);
    for ( int i=0; i<_nNodes; i++)
        file.read(reinterpret_cast<char *>(&_links[i][0]),_nNodes*sizeof(_links[i][0]));

    file.read( (char*)(&_nodes[0]),_nodes.size()*sizeof(_nodes[0]));
}

/**\brief An step in the graph path.
 *
 * Please note that the first element is the origin of the path.*/
struct GraphStep
{
  int index; ///<index of the node
  float weight;///<weight of the link
  float accWeight;///<accumulated weight
};

/**\brief Expresses a path into a graph */
class  GraphPath :public std::vector<GraphStep>
{
  public:
    /** @brief empty constructor.  */
    GraphPath() {}

    /** @brief clears */
    void clear()
    {
      std::vector<GraphStep>::clear();
      totalWeight=0;
    }

    void setOrigin(int index)
    {
      GraphStep GS;
      GS.index=index;
      GS.weight=0;
      totalWeight+=0;
      push_back(GS);
    }

    void addNode(int index,double weight)
    {
      GraphStep GS;
      GS.index=index;
      GS.weight=weight;
      totalWeight+=weight;
      GS.accWeight=totalWeight;
      push_back(GS);
    }

    /** @brief writes an string with the path */
    std::string toString()
    {
      std::stringstream sret;
      for (unsigned int i=0; i<size(); i++)
        sret<<"(node="<<(*this)[i].index<<", weight="<<(*this)[i].weight<<")";
      sret<<" Total weight="<<totalWeight<<std::endl;
      return sret.str();
    }

  public:
    double totalWeight;
};

//El algoritmo encuentra el camino entre todos los pares de vértices en una única ejecución.
class Floyd{
public:
    template<class T>
    void process(const Graph<T> &G){
        dist.resize(G.size());
        for(auto &d:dist)d.resize(G.size());
        next.resize(G.size());
        for(auto &n:next)n.resize(G.size());

        for(int i=0;i<G.size();i++){
            for(int j=0;j<G.size();j++){
                if (i!=j ){
                    dist[i][j]=G(i,j);
                    if (G(i,j)!=G. getInfinity())
                        next[i][j]=j;
                    else   next[i][j]=-1;

                }
                else {
                    dist[i][j]=0;
                }
            }
        }

        for(int k=0;k<G.size();k++)
            for(int i=0;i<G.size();i++)
                for(int j=0;j<G.size();j++){
                    if (dist[i][k]!=G. getInfinity() && dist[k][j]!=G. getInfinity()){
                        double d=dist[i][k]+dist[k][j];
                        if(  d< dist[i][j]){
                            dist[i][j]=d;
                            next[i][j]=next[i][k];
                        }
                    }
                }


    }

    std::vector<uint32_t> path(uint32_t i,uint32_t j)const{
        std::vector<uint32_t> p;
        if (next[i][j]==-1) return p;
        p.push_back(i);
        while(i!=j){
            i=next[i][j];
            p.push_back(i);
        }
        return p;
    }

    //indicates if nodes i and j are connected
    bool areConnected(uint32_t i,uint32_t j)const{
        return (next[i][j]!=-1);
    }

    bool isValid()const{return dist.size()!=0;}
    std::vector<std::vector<double > > dist;
    std::vector<std::vector<int > > next;
};
//Stoer-Wagner min cut algorithm.
class Stoer_Wagner_min_cut {
public:


    template<typename T>
    static  std::pair<int,  std::vector<int> > min_cut(const Graph<T>  &g) {
        const int INF = 1000000000;

        std::vector<std::vector<T> > weights(g.size());
        for(auto &w:weights)w.resize(g.size());
        //set data
        for(int i=0;i<g.size();i++)
            for(int j=0;j<g.size();j++){
                if (g.areConnected(i,j))  weights[i][j]=g(i,j);
                else weights[i][j]=INF;
            }
        return min_cut(weights);
    }


    template<typename T>
    static  std::pair<double,  std::vector<int> > min_cut(std::vector<std::vector<T> >  &weights) {
        int N = weights.size();
        std::vector<bool>  used(N,false);
        std::vector<int> cut, best_cut;
        double best_weight = -1;


        for (int phase = N-1; phase >= 0; phase--) {
            std::vector<T>  w = weights[0];
            std::vector<bool>  added = used;
            int prev, last = 0;
            for (int i = 0; i < phase; i++) {
                prev = last;
                last = -1;
                for (int j = 1; j < N; j++)
                    if (!added[j] && (last == -1 || w[j] > w[last])) last = j;
                if (i == phase-1) {
                    for (int j = 0; j < N; j++) weights[prev][j] += weights[last][j];
                    for (int j = 0; j < N; j++) weights[j][prev] = weights[prev][j];
                    used[last] = true;
                    cut.push_back(last);
                    if (best_weight == -1 || w[last] < best_weight) {
                        best_cut = cut;
                        best_weight = w[last];
                    }
                } else {
                    for (int j = 0; j < N; j++)
                        w[j] += weights[last][j];
                    added[last] = true;
                }
            }
        }
        return make_pair(best_weight, best_cut);
    }
};
/**\brief Class with algorithms to operate in graphs */
class Dijkstra
{



  public:
    struct Info
    {
        Info(){}

        void setSize(unsigned int n){
            _distances.resize(n);
            _predecesors.resize(n);
            _n=n;
        }

      Info(unsigned int n)
      {
          setSize(n);
      }



      std::vector<double> _distances;
      std::vector< int> _predecesors;
      unsigned int _n;
    };

    /** @brief Runs the Dijkstra algorithm on the graph passed using the node
     * indicated.
     *
     * The function returns a pointer to a class that can be used to extract
     * the paths using dijkstraPath() */
    template<class T>
    static Info  dijkstra(const Graph<T> &G, int origin)
    {
      int i, j;
      std::vector<int> s(G.size()); //Vector que indica si el nodo i pertenece al conjunto s
      double minimo;
      int	 x;
      Info di_res;
      di_res.setSize(G.size());



      //Reservamos espacio para s

      //Inicializacion de los vectores de DI->_distancess y DI->_predecesorses
      for (i = 0; i < G.size(); i++)
      {
        s[i] = 0;
        di_res._predecesors[i] = origin;
        di_res._distances[i] = G(origin, i);
      }

      //Al comienzo solo el nodo origin pertenece al conjunto s
      s[origin] = 1;

      //Se itera n-1 veces para introducir el resto de nodos en s
      for (i = 0; i <  G.size()-1; i++)
      {
        minimo = G. getInfinity();
        for (j = 0; j <  G.size(); j++)
        {
          //Se selecciona un nodo que no pertenece a s y su distancia  es minima
          if (s[j] == 0 && di_res._distances[j] < minimo)
          {
            minimo =di_res._distances[j];
            x = j;
          }
        }
        if (minimo!=G. getInfinity())
        {
          s[x] = 1; //Introducimos nodo x en el conjunto s

          //Se corrige el vector de DI->_distancess al entrar x en el conjunto s
          for (j = 0; j <  G.size(); j++)
          {
            if (s[j] == 0 && di_res._distances[j] > di_res._distances[x] + G(x, j))
            {
              di_res._distances[j] = di_res._distances[x] + G(x, j);
             di_res._predecesors[j] = x;
            }
          }
        }
      }
      return di_res;
    }

    /** @brief Calculates the path from the start node to the end one.
     */
    static GraphPath dijkstraPath(const Info&DI, unsigned int start, unsigned int end){
        assert(start<DI._n && end<DI._n );
        std::vector<unsigned int>v;
        dijkstraPath(DI,start,end,v);
      //create the path
        GraphPath GP;
        GP.setOrigin(start);
        double prevW=0;
        for (unsigned int i=0; i<v.size(); i++)
        {
          if (DI._distances[v[i]]==std::numeric_limits<double>::max())
            return GraphPath(); //no path
          GP.addNode(v[i],DI._distances[v[i]]-prevW);
          prevW=DI._distances[v[i]];
        }
        return GP;
    }

  private:
    static void dijkstraPath(const Info &DI, unsigned int start,
                             unsigned int end, std::vector<unsigned int>& v){
        assert(start<DI._n && end<DI._n );
        if (start != end)
        {
          dijkstraPath(DI, start, DI._predecesors[end], v );
          v.push_back(end);
        }
    }
};

};
};
#endif