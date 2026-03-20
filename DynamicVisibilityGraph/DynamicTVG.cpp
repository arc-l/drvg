#include "DynamicRVG.h"
void DynamicRVG<T>::scanVisibleArea() // scan the environment at the current robot location and then update the _visibleAreaInMap
{
    
}

VisibilityGraph DynamicRVG<T>::buildVisibilityGraph() // build the visibility graph based on the _visibleAreaInMap
{

}

void DynamicRVG<T>::mergeVisibilityGraph(const VisibilityGraph &newGraph) // merge the new visibility graph with the existing one  
{

}

bool DynamicRVG<T>::plan(const Point<T> &start, const Point<T> &goal)
{
    /*
        while not find solution
            scanVisibleArea();
            VisibilityGraph newGraph = buildVisibilityGraph();
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
} // plan a path from start to goal using the visibility graph
    
//TODO: We can have several visualizatio functions to help us debugging.
// For example, we can visualize the _visibleAreaInMap, the _explorationPath, the _shortestPath, and the merged visibility graph.
// We can also visualize the RVG built after every step