#ifndef DYNAMIC_VISIBILITY_GRAPH_DYNAMIC_RVG_H
#define DYNAMIC_VISIBILITY_GRAPH_DYNAMIC_RVG_H

#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Graph.h> 
#include <VisibilityGraph/VisibilityGraph.h>

namespace RotationalVisibilityGraph {

template <typename T>
class DynamicRVG{
public:
    DynamicRVG(
        const Polygon<T> &robot,
        const Polygon<T> &border, 
        const std::vector<Polygon<T>> &obstacles, 
        const T &resolution, 
        const int &numThreads
    ): _robot(robot), _border(border), _cameraOffset(0, 0), _obstacles(obstacles), _resolution(resolution), _numThreads(numThreads), _alpha(1), _beta(0){}
    ~DynamicRVG() = default;

    void setCameraOffset(const Vertex<T> &cameraOffset); // set the camera offset from the robot center, which is used to calculate the visible area. We assume the camera is always facing forward and the offset is along the forward direction of the robot.

    const Polygon<T> &scanVisibleArea(const Vertex<T> & currentLocation); // scan the environment at the current robot location and return the visible polygon

    std::shared_ptr<Vertex<T>> calculateTemporaryGoal(const std::shared_ptr<Vertex<T>> &currentVertex) const; // calculate a temporary goal for the robot to move to when the real goal is not visible. We rank discovered vertices by cost from the real start plus an estimate to the real goal, and we never allow the current robot position itself to be chosen again.

    void calculateShortestPath(const std::shared_ptr<Vertex<T>> &temporaryGoal); // calculate the shortest path from the start to the temporary goal based on the current visibility graph. 

    bool plan(const std::shared_ptr<Vertex<T>> &start, const std::shared_ptr<Vertex<T>> &goal);
    bool planIncrementalMapping(const std::shared_ptr<Vertex<T>> &start, const std::shared_ptr<Vertex<T>> &goal); // plan using a persistent explored free-space map assembled from all scans so far.
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
    void drawVisibleArea(const std::string &name = "") const; // visualize the _visibleAreaInMap
    std::string drawIteration(const std::string &name = "", const std::shared_ptr<Vertex<T>> &temporaryGoal = nullptr) const;
    std::string drawFullPathAndEndGraph(const std::string &name = "") const;

    //testing functions
    void moverobotdebug(const Vertex<T> &nextLocation); // move the robot to the next location
    const std::vector<std::shared_ptr<Vertex<T>>>& getExplorationPath() const; 
    const std::vector<std::shared_ptr<Vertex<T>>>& getShortestPath() const; 
    const Graph<T> & getGraph() const;
    void setGoal(const std::shared_ptr<Vertex<T>> &goal);
    void setGraph(const Graph<T>& graph);
    void setWeight(T alpha, T beta);

private:
    Vertex<T> getScanLocation(const Vertex<T> &robotCenter) const;

    Polygon<T> _robot, _border, _realRobot;
    Vertex<T> _cameraOffset; // the offset of the camera from the robot center, which is used to calculate the visible area. We assume the camera is always facing forward and the offset is along the forward direction of the robot.
    std::vector<Polygon<T>> _obstacles;
    int _resolution;
    int _numThreads;
    T _alpha, _beta;
    Graph<T> _graph;
    Polygon<T> _visibleAreaInMap;
    std::shared_ptr<Vertex<T>> _start, _goal;
    std::unordered_set<std::shared_ptr<Vertex<T>>, typename Vertex<T>::SharedPtrVertexHash, typename Vertex<T>::SharedPtrVertexEqual> _exploredVertices;
    std::vector<std::shared_ptr<Vertex<T>>> _explorationPath;
    std::vector<std::shared_ptr<Vertex<T>>> _shortestPath; // The shortest path from the start and goal based the current RVG, which is gradually built.
};

}
#endif // DYNAMIC_VISIBILITY_GRAPH_DYNAMIC_RVG_H
