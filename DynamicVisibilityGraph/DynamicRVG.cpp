#include "DynamicRVG.h"
namespace RotationalVisibilityGraph {
    
template <typename T>
void DynamicRVG<T>::scanVisibleArea() // scan the environment at the current robot location and then update the _visibleAreaInMap
{
    
}

template <typename T>
VisibilityGraph<T> DynamicRVG<T>::buildVisibilityGraph() // build the visibility graph based on the _visibleAreaInMap
{

}

template <typename T>
void DynamicRVG<T>::mergeVisibilityGraph(const VisibilityGraph<T> &newGraph) // merge the new visibility graph with the existing one  
{

}

template <typename T>
bool DynamicRVG<T>::plan(const Vertex<T> &start, const Vertex<T> &goal)
{
    /*
        while not find solution
            scanVisibleArea();
            VisibilityGraph<T> newGraph = buildVisibilityGraph();
            mergeVisibilityGraph(newGraph);
            bool goalLegal = _visibilityGraph.addVertex(std::make_shared<Vertex<T>>(goal));
            if (iter==1 && goalLegal){
              check whether the start and the goal can be directly connected, if yes, return the path 
            }
            if (goalLegal){
                update the path from start to goal, including _shortestPath and _explorationPath, and return true if find a path
                return true;
            } 
            move to next location. Let's use a gready strategy for now by moving to the vertex in the graph that is closest to the goal and not yet explored.
            Later we can use A* for this. For example, cost = cost to start + estimated distance to goal.
            })
        */ 
    return false;
} // plan a path from start to goal using the visibility graph
    
template class DynamicRVG<double>;
template class DynamicRVG<float>;
}
//TODO: We can have several visualization functions to help us debugging.
// For example, we can visualize the _visibleAreaInMap, the _explorationPath, the _shortestPath, and the merged visibility graph.
// We can also visualize the RVG built after every step