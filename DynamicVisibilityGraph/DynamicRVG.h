#ifndef DYNAMIC_VISIBILITY_GRAPH_DYNAMIC_RVG_H
#define DYNAMIC_VISIBILITY_GRAPH_DYNAMIC_RVG_H

#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Graph.h> 
#include <VisibilityGraph/VisibilityGraph.h>

namespace RotationalVisibilityGraph {

template <typename T>
class DynamicRVG : public VisibilityGraph<T> {
public:
    /* TODO:
     * Constructor: add robot, border, obstacles resolution, number of threads.
    */
    DynamicRVG(
        const Polygon<T> &robot,
        const Polygon<T> &border, 
        const std::vector<Polygon<T>> &obstacles, 
        const T &resolution, 
        const int &numThreads
    )
    : VisibilityGraph<T>(robot, border, obstacles, resolution, false, numThreads) {};
    ~DynamicRVG() = default;

    void scanVisibleArea(); // scan the environment at the current robot location and then update the _visibleAreaInMap
    VisibilityGraph<T> buildVisibilityGraph(); // build the visibility graph based on the _visibleAreaInMap
    void mergeVisibilityGraph(const VisibilityGraph<T> &newGraph); // merge the new visibility graph with the existing one  
    bool plan(const Vertex<T> &start, const Vertex<T> &goal);
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
        // plan a path from start to goal using the visibility graph
    
    //TODO: We can have several visualizatio functions to help us debugging.
    // For example, we can visualize the _visibleAreaInMap, the _explorationPath, the _shortestPath, and the merged visibility graph.
    // We can also visualize the RVG built after every step

    //visualization functions
    void drawVisibleArea() const; // visualize the _visibleAreaInMap

private:
    Polygon<T> _visibleAreaInMap;
    Graph<T> _graph;
    std::unordered_set<std::shared_ptr<Vertex<T>>, typename Vertex<T>::SharedPtrVertexHash, typename Vertex<T>::SharedPtrVertexEqual> _exploredVertices;
    std::vector<std::shared_ptr<Vertex<T>>> _explorationPath;
    std::vector<std::shared_ptr<Vertex<T>>> _shortestPath; // The shortest path from the start and goal based the current RVG, which is gradually built.
};

}
#endif // DYNAMIC_VISIBILITY_GRAPH_DYNAMIC_RVG_H