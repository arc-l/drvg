#ifndef DYNAMIC_VISIBILITY_GRAPH_DYNAMIC_RVG_H
#define DYNAMIC_VISIBILITY_GRAPH_DYNAMIC_RVG_H

#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Graph.h> 
#include <VisibilityGraph/VisibilityGraph.h>

namespace RotationalVisibilityGraph {

template <typename T>
class DynamicRVG{
    DECL_CGAL_CARTESIAN_TYPES_T
    DECL_CGAL_POLYGON_TYPES_T
    DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

    static Arrangement_2 buildScanEnvironment(
        const Polygon<T> &border,
        const std::vector<Polygon<T>> &obstacles
    ) {
        std::vector<Polygon_2> map;
        map.reserve(obstacles.size() + 1);
        for (const auto &obstacle : obstacles) {
            map.push_back(obstacle.getPolygon());
        }
        map.push_back(border.getPolygon());
        return obsToArrangement<T>(map);
    }

public:
    DynamicRVG(
        const Polygon<T> &robot,
        const Polygon<T> &border, 
        const std::vector<Polygon<T>> &obstacles, 
        const T &resolution, 
        const int &numThreads
    ): _robot(robot),
       _border(border),
       _obstacles(obstacles),
       _scanEnvironment(buildScanEnvironment(border, obstacles)),
       _pointLocation(_scanEnvironment),
       _visibility(_scanEnvironment),
       _resolution(resolution),
       _numThreads(numThreads),
       _alpha(1),
       _beta(0){}
    ~DynamicRVG() = default;

    const Polygon<T> &scanVisibleArea(const Vertex<T> & currentLocation); // scan the environment at the current robot location from the robot center and return the visible polygon
    const Polygon<T> &scanFromAllVertices(const Vertex<T> & currentLocation); // scan the environment from every vertex of the placed robot footprint and merge the visible regions

    std::shared_ptr<Vertex<T>> calculateTemporaryGoal(const std::shared_ptr<Vertex<T>> &currentVertex) const; // calculate a temporary goal for the robot to move to when the real goal is not visible. We rank discovered vertices by cost from the real start plus an estimate to the real goal, and we never allow the current robot position itself to be chosen again.

    void calculateShortestPath(const std::shared_ptr<Vertex<T>> &temporaryGoal); // calculate the shortest path from the start to the temporary goal based on the current visibility graph. 

    bool plan(
        const std::shared_ptr<Vertex<T>> &start,
        const std::shared_ptr<Vertex<T>> &goal,
        bool useScanFromAllVertices = false
    );
    bool planIncrementalMapping(
        const std::shared_ptr<Vertex<T>> &start,
        const std::shared_ptr<Vertex<T>> &goal,
        bool useScanFromAllVertices = false
    ); // plan using a persistent explored free-space map assembled from all scans so far.
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
    void setWriteIterationVisualizations(bool enabled);
    void setIterationVisualizationTag(const std::string &tag);

private:
    Polygon<T> _robot, _border, _realRobot;
    std::vector<Polygon<T>> _obstacles;
    Arrangement_2 _scanEnvironment;
    PL_2 _pointLocation;
    VQ _visibility;
    int _resolution;
    int _numThreads;
    T _alpha, _beta;
    Graph<T> _graph;
    Polygon<T> _visibleAreaInMap;
    Polygon_with_holes_2 _visibleAreaWithHoles;
    bool _hasVisibleAreaWithHoles = false;
    Polygon<T> _mappedBorder;
    std::vector<Polygon<T>> _mappedObstacles;
    std::shared_ptr<Vertex<T>> _start, _goal;
    std::unordered_set<std::shared_ptr<Vertex<T>>, typename Vertex<T>::SharedPtrVertexHash, typename Vertex<T>::SharedPtrVertexEqual> _exploredVertices;
    std::vector<std::shared_ptr<Vertex<T>>> _explorationPath;
    std::vector<std::shared_ptr<Vertex<T>>> _shortestPath; // The shortest path from the start and goal based the current RVG, which is gradually built.
    std::shared_ptr<Vertex<T>> _debugCurrentLocation;
    std::vector<Vertex<T>> _debugScanLocations;
    std::vector<Polygon<T>> _debugVisibleAreas;
    std::vector<Polygon_with_holes_2> _debugMergedVisibleAreas;
    Polygon<T> _debugIterationBorder;
    std::vector<Polygon<T>> _debugIterationObstacles;
    Graph<T> _debugIterationGraph;
    bool _debugStartAddedToIterationGraph = false;
    bool _debugGoalAddedToIterationGraph = false;
    bool _debugDirectGoalConnection = false;
    bool _writeIterationVisualizations = false;
    std::string _iterationVisualizationTag;
};

}
#endif // DYNAMIC_VISIBILITY_GRAPH_DYNAMIC_RVG_H
