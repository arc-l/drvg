#include "DynamicRVG.h"
#include <VisibilityGraph/Utils.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iterator>
#include <queue>
#include <Utils/Utils.h>

namespace RotationalVisibilityGraph {

namespace {

DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T

template <typename Point2, typename Polygon2>
bool sanitizePolygonPointsForBooleanOps(
    const std::vector<Point2> &inputPoints,
    Polygon2 &output
) {
    output.clear();
    if (inputPoints.size() < 3) {
        return false;
    }

    std::vector<Point2> points;
    points.reserve(inputPoints.size());
    for (const auto &point : inputPoints) {
        if (!points.empty() && point == points.back()) {
            continue;
        }
        points.push_back(point);
    }
    while (points.size() >= 2 && points.front() == points.back()) {
        points.pop_back();
    }
    if (points.size() < 3) {
        return false;
    }

    bool removedPoint = true;
    while (removedPoint && points.size() >= 3) {
        removedPoint = false;
        std::vector<Point2> simplified;
        simplified.reserve(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            const Point2 &prev = points[(i + points.size() - 1) % points.size()];
            const Point2 &curr = points[i];
            const Point2 &next = points[(i + 1) % points.size()];
            if (CGAL::orientation(prev, curr, next) == CGAL::COLLINEAR) {
                removedPoint = true;
                continue;
            }
            if (!simplified.empty() && curr == simplified.back()) {
                removedPoint = true;
                continue;
            }
            simplified.push_back(curr);
        }
        points.swap(simplified);
    }

    if (points.size() < 3) {
        return false;
    }

    Polygon2 candidate(points.begin(), points.end());
    if (candidate.is_clockwise_oriented()) {
        candidate.reverse_orientation();
    }
    if (!candidate.is_simple()) {
        return false;
    }
    if (candidate.orientation() != CGAL::COUNTERCLOCKWISE) {
        return false;
    }
    if (candidate.area() == 0) {
        return false;
    }

    output = candidate;
    return true;
}

template <typename T>
bool sanitizePolygonForBooleanOps(
    const Polygon<T> &input,
    typename Polygon<T>::Polygon_2 &output
) {
    using Point2 = typename Polygon<T>::Point_2;

    std::vector<Point2> points;
    points.reserve(input.size());
    for (const auto &vertex : input.getVertices()) {
        points.push_back(vertex.getPoint());
    }
    return sanitizePolygonPointsForBooleanOps(points, output);
}

template <typename Point2, typename PolygonWithHoles>
bool pointInsidePolygonWithHoles(
    const Point2 &point,
    const PolygonWithHoles &polygonWithHoles
) {
    const auto &outerBoundary = polygonWithHoles.outer_boundary();
    if (!(IN_POLYGON(point, outerBoundary) || ON_EDGE(point, outerBoundary))) {
        return false;
    }
    for (auto holeIt = polygonWithHoles.holes_begin(); holeIt != polygonWithHoles.holes_end(); ++holeIt) {
        const auto &hole = *holeIt;
        if (IN_POLYGON(point, hole) || ON_EDGE(point, hole)) {
            return false;
        }
    }
    return true;
}

template <typename T, typename Arrangement2, typename PointLocation, typename VisibilityQuery>
bool extractVisibleAreaFromScanPoint(
    PointLocation &pointLocation,
    VisibilityQuery &visibility,
    const Vertex<T> &scanLocation,
    typename Polygon<T>::Polygon_2 &visibleArea
)
{
    Arrangement2 visibleAreaArrangement;
    auto obj = pointLocation.locate(scanLocation.getPoint());
    auto face = boost::get<typename Arrangement2::Face_const_handle>(&obj);
    if (!face) {
        return false;
    }

    auto visibleFace = visibility.compute_visibility(scanLocation.getPoint(), *face, visibleAreaArrangement);
    std::vector<typename Polygon<T>::Point_2> boundary;
    auto edge = visibleFace->outer_ccb();
    do {
        boundary.push_back(edge->source()->point());
        ++edge;
    } while (edge != visibleFace->outer_ccb());
    return sanitizePolygonPointsForBooleanOps(boundary, visibleArea);
}

inline std::vector<Polygon_with_holes_2> mergeVisibleAreasWithHoles(
    const std::vector<Polygon_2> &visibleAreas
)
{
    Polygon_set_2 visibleAreaUnion;
    for (const auto &visibleArea : visibleAreas) {
        visibleAreaUnion.join(visibleArea);
    }

    std::vector<Polygon_with_holes_2> mergedRegions;
    visibleAreaUnion.polygons_with_holes(std::back_inserter(mergedRegions));
    return mergedRegions;
}

template <typename T>
size_t graphEdgeCount(const Graph<T> &graph)
{
    size_t degreeSum = 0;
    for (const auto &entry : graph.getAdjacencyList()) {
        degreeSum += entry.second.size();
    }
    return degreeSum / 2;
}

template <typename T>
bool graphHasEdge(
    const Graph<T> &graph,
    const std::shared_ptr<Vertex<T>> &from,
    const std::shared_ptr<Vertex<T>> &to
)
{
    if (!from || !to) {
        return false;
    }
    const auto &adjacencyList = graph.getAdjacencyList();
    const auto fromIt = adjacencyList.find(from);
    if (fromIt == adjacencyList.end()) {
        return false;
    }
    return fromIt->second.find(to) != fromIt->second.end();
}

}

template <typename T>
const Polygon<T> &DynamicRVG<T>::scanVisibleArea(const Vertex<T> & currentLocation) // scan the environment at the current robot location and then update the _visibleAreaInMap
{
    _debugCurrentLocation = std::make_shared<Vertex<T>>(currentLocation);
    _debugScanLocations = {currentLocation};
    _debugVisibleAreas.clear();
    _debugMergedVisibleAreas.clear();
    _debugIterationBorder = Polygon<T>();
    _debugIterationObstacles.clear();
    _debugIterationGraph = Graph<T>();
    _debugStartAddedToIterationGraph = false;
    _debugGoalAddedToIterationGraph = false;
    _debugDirectGoalConnection = false;
    _hasVisibleAreaWithHoles = false;
    _visibleAreaWithHoles = Polygon_with_holes_2();
    _mappedBorder = Polygon<T>();
    _mappedObstacles.clear();

    Polygon_2 visibleArea;
    if (!extractVisibleAreaFromScanPoint<T, Arrangement_2>(_pointLocation, _visibility, currentLocation, visibleArea)) {
        _visibleAreaInMap = Polygon<T>();
        return _visibleAreaInMap;
    }
    _visibleAreaInMap = Polygon<T>(visibleArea);
    _visibleAreaWithHoles = Polygon_with_holes_2(visibleArea);
    _hasVisibleAreaWithHoles = true;
    _mappedBorder = _visibleAreaInMap;
    _debugVisibleAreas.push_back(_visibleAreaInMap);
    _debugMergedVisibleAreas.push_back(_visibleAreaWithHoles);
    return _visibleAreaInMap;
}

template <typename T>
const Polygon<T> &DynamicRVG<T>::scanFromAllVertices(const Vertex<T> & currentLocation)
{
    _debugCurrentLocation = std::make_shared<Vertex<T>>(currentLocation);
    _debugScanLocations.clear();
    _debugVisibleAreas.clear();
    _debugMergedVisibleAreas.clear();
    _debugIterationBorder = Polygon<T>();
    _debugIterationObstacles.clear();
    _debugIterationGraph = Graph<T>();
    _debugStartAddedToIterationGraph = false;
    _debugGoalAddedToIterationGraph = false;
    _debugDirectGoalConnection = false;
    _hasVisibleAreaWithHoles = false;
    _visibleAreaWithHoles = Polygon_with_holes_2();
    _mappedBorder = Polygon<T>();
    _mappedObstacles.clear();

    std::vector<Polygon_2> visibleAreas;
    const T robotTheta = currentLocation.hasTheta() ? currentLocation.getTheta() : static_cast<T>(0);
    const Polygon<T> robotFootprint = this->_robot.moveToCopy(
        currentLocation.getX(),
        currentLocation.getY(),
        robotTheta
    );
    const T scanInsetFraction = static_cast<T>(1e-6);

    for (const auto &robotVertex : robotFootprint.getVertices()) {
        Vertex<T> scanLocation(robotVertex);
        const T dx = currentLocation.getX() - robotVertex.getX();
        const T dy = currentLocation.getY() - robotVertex.getY();
        scanLocation.setPos(
            robotVertex.getX() + dx * scanInsetFraction,
            robotVertex.getY() + dy * scanInsetFraction
        );
        _debugScanLocations.push_back(scanLocation);

        Polygon_2 visibleAreaAtVertex;
        if (!extractVisibleAreaFromScanPoint<T, Arrangement_2>(_pointLocation, _visibility, scanLocation, visibleAreaAtVertex)) {
            continue;
        }
        visibleAreas.push_back(visibleAreaAtVertex);
        _debugVisibleAreas.emplace_back(visibleAreaAtVertex);
    }

    if (visibleAreas.empty()) {
        _visibleAreaInMap = Polygon<T>();
        return _visibleAreaInMap;
    }

    std::vector<Polygon_with_holes_2> visibleRegions = mergeVisibleAreasWithHoles(visibleAreas);
    _debugMergedVisibleAreas = visibleRegions;
    if (visibleRegions.empty()) {
        _visibleAreaInMap = Polygon<T>();
        return _visibleAreaInMap;
    }

    const auto containsRobot = [&](const Polygon_with_holes_2 &region) {
        if (pointInsidePolygonWithHoles(currentLocation.getPoint(), region)) {
            return true;
        }
        return std::any_of(
            robotFootprint.getVertices().begin(),
            robotFootprint.getVertices().end(),
            [&](const Vertex<T> &robotVertex) {
                return pointInsidePolygonWithHoles(robotVertex.getPoint(), region);
            }
        );
    };
    const auto regionIt = std::find_if(visibleRegions.begin(), visibleRegions.end(), containsRobot);
    const Polygon_with_holes_2 &selectedRegion = regionIt != visibleRegions.end()
        ? *regionIt
        : visibleRegions.front();

    if (selectedRegion.outer_boundary().size() < 3) {
        _visibleAreaInMap = Polygon<T>();
        return _visibleAreaInMap;
    }

    _visibleAreaWithHoles = selectedRegion;
    _hasVisibleAreaWithHoles = true;
    _visibleAreaInMap = Polygon<T>(selectedRegion.outer_boundary());
    _mappedBorder = _visibleAreaInMap;
    _mappedObstacles.clear();
    for (auto holeIt = selectedRegion.holes_begin(); holeIt != selectedRegion.holes_end(); ++holeIt) {
        _mappedObstacles.emplace_back(*holeIt);
    }
    return _visibleAreaInMap;
}

template <typename T>
std::shared_ptr<Vertex<T>> DynamicRVG<T>::calculateTemporaryGoal(const std::shared_ptr<Vertex<T>> &currentVertex) const
{
    if (!_start || !_goal || !currentVertex) {
        std::cout << "Start, goal, or current vertex is not set." << std::endl;
        return nullptr;
    }

    const auto &vertices = _graph.getVertices();
    if (vertices.empty()) {
        std::cout << "Visibility graph has no vertices." << std::endl;
        return nullptr;
    }

    const auto &adjacencyList = _graph.getAdjacencyList();
    if (adjacencyList.find(this->_start) == adjacencyList.end()) {
        std::cout << "Start vertex is not in the visibility graph." << std::endl;
        return nullptr;
    }

    const auto edgeCost = [&](const std::shared_ptr<Vertex<T>> &from, const std::shared_ptr<Vertex<T>> &to) {
        return _alpha * from->dist(*to) + _beta * from->rotationalDist(*to);
    };

    std::unordered_map<
        std::shared_ptr<Vertex<T>>,
        T,
        typename Graph<T>::SharedPtrVertexHash,
        typename Graph<T>::SharedPtrVertexEqual
    > distanceFromRealStart;
    for (const auto &vertex : vertices) {
        distanceFromRealStart[vertex] = std::numeric_limits<T>::max();
    }

    std::priority_queue<
        std::pair<T, std::shared_ptr<Vertex<T>>>,
        std::vector<std::pair<T, std::shared_ptr<Vertex<T>>>>,
        std::greater<>
    > frontier;
    distanceFromRealStart[this->_start] = 0;
    frontier.push({0, this->_start});
    while (!frontier.empty()) {
        const auto [currentCost, current] = frontier.top();
        frontier.pop();
        if (currentCost > distanceFromRealStart[current]) {
            continue;
        }
        for (const auto &neighbor : adjacencyList.at(current)) {
            const T nextCost = currentCost + edgeCost(current, neighbor);
            if (nextCost < distanceFromRealStart[neighbor]) {
                distanceFromRealStart[neighbor] = nextCost;
                frontier.push({nextCost, neighbor});
            }
        }
    }

    const T goalBiasLambda = static_cast<T>(0.0);
    std::shared_ptr<Vertex<T>> bestVertex = nullptr;
    T bestEstimatedTotalCost = std::numeric_limits<T>::max();
    const auto isExploredPosition = [&](const std::shared_ptr<Vertex<T>> &candidate) {
        for (const auto &exploredVertex : _exploredVertices) {
            if (candidate->dist(*exploredVertex) < 1e-5) {
                return true;
            }
        }
        return false;
    };
    for (const auto &vertex : vertices) {
        const T costFromRealStart = distanceFromRealStart[vertex];
        if (costFromRealStart == std::numeric_limits<T>::max()) {
            continue;
        }
        if (vertex->dist(*currentVertex) < 1e-5) {
            continue;
        }
        if (isExploredPosition(vertex)) {
            continue;
        }
        const T estimatedCostToGoal = edgeCost(vertex, this->_goal);
        const T estimatedTotalCost = goalBiasLambda * costFromRealStart + estimatedCostToGoal;
        if (estimatedTotalCost < bestEstimatedTotalCost) {
            bestEstimatedTotalCost = estimatedTotalCost;
            bestVertex = vertex;
        }
    }

    return bestVertex;
}


template <typename T>
void DynamicRVG<T>::calculateShortestPath(const std::shared_ptr<Vertex<T>> &temporaryGoal)
{
    _explorationPath.clear();
    _shortestPath.clear();

    if (!temporaryGoal || !_start || !_goal) {
        return;
    }

    const auto appendPath = [&](const std::vector<Vertex<T>> &path) {
        for (const auto &vertex : path) {
            const auto vertexPtr = std::make_shared<Vertex<T>>(vertex);
            _explorationPath.push_back(vertexPtr);
            _shortestPath.push_back(vertexPtr);
        }
    };

    const Point_2 goalPoint = this->_goal->getPoint();
    const bool goalVisible = _hasVisibleAreaWithHoles
        ? pointInsidePolygonWithHoles(goalPoint, _visibleAreaWithHoles)
        : (_visibleAreaInMap.size() &&
           (IN_POLYGON(goalPoint, _visibleAreaInMap.getPolygon()) ||
            ON_EDGE(goalPoint, _visibleAreaInMap.getPolygon())));
    if (goalVisible && this->_goal->dist(*temporaryGoal) < 1e-5) {
        appendPath(std::vector<Vertex<T>>{*this->_start, *this->_goal});
        return;
    }

    const std::vector<Vertex<T>> path = _graph.shortestPath(this->_start, temporaryGoal, false);
    appendPath(path);
    if (_explorationPath.empty()) {
        Utils::print("No path found from start to temporary goal");
    }
}

template <typename T>
bool DynamicRVG<T>::plan(
    const std::shared_ptr<Vertex<T>> &start,
    const std::shared_ptr<Vertex<T>> &goal,
    bool useScanFromAllVertices
)
{
    this->_start = start;
    this->_goal = goal;
    if (!this->_start || !this->_goal) {
        return false;
    }
    _visibleAreaInMap = Polygon<T>();
    _visibleAreaWithHoles = Polygon_with_holes_2();
    _hasVisibleAreaWithHoles = false;
    _mappedBorder = Polygon<T>();
    _mappedObstacles.clear();
    _graph = Graph<T>();
    _graph.setWeight(_alpha, _beta);
    _exploredVertices.clear();
    _exploredVertices.insert(this->_start);
    _explorationPath.clear();
    _shortestPath.clear();
    _debugCurrentLocation.reset();
    _debugScanLocations.clear();
    _debugVisibleAreas.clear();
    _debugMergedVisibleAreas.clear();
    _debugIterationBorder = Polygon<T>();
    _debugIterationObstacles.clear();
    _debugIterationGraph = Graph<T>();
    _debugStartAddedToIterationGraph = false;
    _debugGoalAddedToIterationGraph = false;
    _debugDirectGoalConnection = false;

    std::vector<std::shared_ptr<Vertex<T>>> fullExplorationPath;
    const auto appendSegment = [&](const std::vector<Vertex<T>> &segment) {
        for (size_t i = 0; i < segment.size(); ++i) {
            if (!fullExplorationPath.empty() && i == 0 && *fullExplorationPath.back() == segment[i]) {
                continue;
            }
            fullExplorationPath.push_back(std::make_shared<Vertex<T>>(segment[i]));
        }
    };
    size_t failureDrawCounter = 0;
    const auto drawFailureState = [&](int iteration, const std::string &reason, const std::shared_ptr<Vertex<T>> &temporaryGoal = nullptr) {
        _explorationPath = fullExplorationPath;
        _shortestPath = fullExplorationPath;
        if (!_writeIterationVisualizations) {
            return;
        }
        ++failureDrawCounter;
        const std::string tagPrefix = _iterationVisualizationTag.empty() ? "" : _iterationVisualizationTag + "_";
        std::string name = tagPrefix + "failed_" + std::to_string(failureDrawCounter) + "_iter_" + std::to_string(iteration + 1) + "_" + reason;
        const std::string scriptPath = drawIteration(name, temporaryGoal);
        RotationalVisibilityGraph::Utils::runPythonScriptAndRemove<T>(scriptPath);
    };
    const auto drawCurrentIteration = [&](int iteration, const std::shared_ptr<Vertex<T>> &temporaryGoal = nullptr) {
        if (!_writeIterationVisualizations) {
            return;
        }
        const std::string tagPrefix = _iterationVisualizationTag.empty() ? "" : _iterationVisualizationTag + "_";
        std::string name = tagPrefix + "iter_" + std::to_string(iteration + 1);
        const std::string scriptPath = drawIteration(name, temporaryGoal);
        RotationalVisibilityGraph::Utils::runPythonScriptAndRemove<T>(scriptPath);
    };

    const int maxIterations = 10000;
    auto tempStart = this->_start;
    auto tempGoal = this->_goal;
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        if (useScanFromAllVertices) {
            scanFromAllVertices(*tempStart);
        } else {
            scanVisibleArea(*tempStart);
        }
        VisibilityGraph<T> iterationVG(
            this->_robot,
            this->_mappedBorder.size() ? this->_mappedBorder : this->_visibleAreaInMap,
            this->_mappedObstacles,
            this->_resolution,
            false,
            this->_numThreads
        );
        bool startAdded = iterationVG.addVertex(tempStart);
        // const auto &layers = iterationVG.getLayers();
        // for (size_t i = 0; i < layers.size(); ++i){
        //     std::string layerFigPath = "iteration" + std::to_string(iteration) + "layer-" + std::to_string(i) + ".png";
        //     const auto & layer = layers[i];
        //     layer.draw(layerFigPath, this->_visibleAreaInMap, this->_obstacles, nullptr, false);
        // }
        (void) startAdded;
        bool goalAdded = iterationVG.addVertex(this->_goal);
        _debugCurrentLocation = tempStart;
        _debugIterationBorder = this->_mappedBorder.size() ? this->_mappedBorder : this->_visibleAreaInMap;
        _debugIterationObstacles = this->_mappedObstacles;
        _debugIterationGraph = iterationVG.getGraph();
        _debugStartAddedToIterationGraph = startAdded;
        _debugGoalAddedToIterationGraph = goalAdded;
        _debugDirectGoalConnection = startAdded && goalAdded &&
            graphHasEdge(_debugIterationGraph, tempStart, this->_goal);
        _graph.mergeGraph(iterationVG.getGraph());
        if (goalAdded){
            const auto shortestPath = _graph.shortestPath(
                tempStart,
                this->_goal,
                false
            );
            if (!shortestPath.empty()) {
                appendSegment(shortestPath);
                _explorationPath = fullExplorationPath;
                const auto realShortestPath = _graph.shortestPath(
                    this->_start,
                    this->_goal,
                    false
                );
                for (const auto &vertex : realShortestPath) {
                    const auto vertexPtr = std::make_shared<Vertex<T>>(vertex);
                    _shortestPath.push_back(vertexPtr);
                }
                drawCurrentIteration(iteration, this->_goal);
                return true;
            }
        }
        std::cout << "Iteration " << iteration + 1 << ": _alpha = " << _alpha << ", _beta = " << _beta << std::endl;
        tempGoal = calculateTemporaryGoal(tempStart);
        std::cout << "Temporary goal: " << *tempGoal << std::endl;
        if (!tempGoal) {
            Utils::print("No temporary goal found in this iteration, stopping planning.");
            drawFailureState(iteration, "no_temporary_goal");
            return false;
        }

        _graph.setWeight(_alpha, _beta);
        const auto shortestPath = _graph.shortestPath(
            tempStart,
            tempGoal,
            false
        );
        if (shortestPath.empty()) {
            Utils::print("No path found in this iteration, stopping planning.");
            drawFailureState(iteration, "no_path", tempGoal);
            return false;
        }

        appendSegment(shortestPath);
        _explorationPath = fullExplorationPath;
        drawCurrentIteration(iteration, tempGoal);
        _exploredVertices.insert(tempGoal);
        tempStart = tempGoal;
    }

    Utils::print("Planning did not converge within the iteration limit.");
    drawFailureState(maxIterations - 1, "iteration_limit");
    return false;
} // plan a path from start to goal using the visibility graph

template <typename T>
bool DynamicRVG<T>::planIncrementalMapping(
    const std::shared_ptr<Vertex<T>> &start,
    const std::shared_ptr<Vertex<T>> &goal,
    bool useScanFromAllVertices
)
{
    DECL_CGAL_CARTESIAN_TYPES_T
    DECL_CGAL_POLYGON_TYPES_T

    this->_start = start;
    this->_goal = goal;
    if (!this->_start || !this->_goal) {
        return false;
    }
    _visibleAreaInMap = Polygon<T>();
    _visibleAreaWithHoles = Polygon_with_holes_2();
    _hasVisibleAreaWithHoles = false;
    _mappedBorder = Polygon<T>();
    _mappedObstacles.clear();
    _graph = Graph<T>();
    _graph.setWeight(_alpha, _beta);
    _exploredVertices.clear();
    _exploredVertices.insert(this->_start);
    _explorationPath.clear();
    _shortestPath.clear();
    _debugCurrentLocation.reset();
    _debugScanLocations.clear();
    _debugVisibleAreas.clear();
    _debugMergedVisibleAreas.clear();
    _debugIterationBorder = Polygon<T>();
    _debugIterationObstacles.clear();
    _debugIterationGraph = Graph<T>();
    _debugStartAddedToIterationGraph = false;
    _debugGoalAddedToIterationGraph = false;
    _debugDirectGoalConnection = false;

    Polygon_with_holes_2 knownMap;
    bool knownMapInitialized = false;
    std::vector<std::shared_ptr<Vertex<T>>> fullExplorationPath;
    const auto appendSegment = [&](const std::vector<Vertex<T>> &segment) {
        for (size_t i = 0; i < segment.size(); ++i) {
            if (!fullExplorationPath.empty() && i == 0 && *fullExplorationPath.back() == segment[i]) {
                continue;
            }
            fullExplorationPath.push_back(std::make_shared<Vertex<T>>(segment[i]));
        }
    };
    size_t failureDrawCounter = 0;
    const auto drawFailureState = [&](int iteration, const std::string &reason, const std::shared_ptr<Vertex<T>> &temporaryGoal = nullptr) {
        _explorationPath = fullExplorationPath;
        _shortestPath = fullExplorationPath;
        if (!_writeIterationVisualizations) {
            return;
        }
        ++failureDrawCounter;
        const std::string tagPrefix = _iterationVisualizationTag.empty() ? "" : _iterationVisualizationTag + "_";
        std::string name = tagPrefix + "incremental_failed_" + std::to_string(failureDrawCounter) + "_iter_" +
                           std::to_string(iteration + 1) + "_" + reason;
        const std::string scriptPath = drawIteration(name, temporaryGoal);
        RotationalVisibilityGraph::Utils::runPythonScriptAndRemove<T>(scriptPath);
    };
    const auto drawCurrentIteration = [&](int iteration, const std::shared_ptr<Vertex<T>> &temporaryGoal = nullptr) {
        if (!_writeIterationVisualizations) {
            return;
        }
        const std::string tagPrefix = _iterationVisualizationTag.empty() ? "" : _iterationVisualizationTag + "_";
        std::string name = tagPrefix + "incremental_iter_" + std::to_string(iteration + 1);
        const std::string scriptPath = drawIteration(name, temporaryGoal);
        RotationalVisibilityGraph::Utils::runPythonScriptAndRemove<T>(scriptPath);
    };
    const auto extractKnownComponent = [&](const Point_2 &queryPoint, Polygon<T> &knownBorder, std::vector<Polygon<T>> &knownObstacles) {
        if (!knownMapInitialized || !pointInsidePolygonWithHoles(queryPoint, knownMap)) {
            return false;
        }

        knownBorder = Polygon<T>(knownMap.outer_boundary());
        knownObstacles.clear();
        for (auto holeIt = knownMap.holes_begin(); holeIt != knownMap.holes_end(); ++holeIt) {
            knownObstacles.emplace_back(*holeIt);
        }
        return true;
    };

    const int maxIterations = 10000;
    auto tempStart = this->_start;
    auto tempGoal = this->_goal;
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        if (useScanFromAllVertices) {
            scanFromAllVertices(*tempStart);
        } else {
            scanVisibleArea(*tempStart);
        }
        if (_hasVisibleAreaWithHoles) {
            if (!knownMapInitialized) {
                knownMap = _visibleAreaWithHoles;
                knownMapInitialized = true;
            } else if (pointInsidePolygonWithHoles(tempStart->getPoint(), knownMap) &&
                       pointInsidePolygonWithHoles(tempStart->getPoint(), _visibleAreaWithHoles)) {
                Polygon_with_holes_2 mergedMap;
                CGAL::join(knownMap, _visibleAreaWithHoles, mergedMap);
                knownMap = mergedMap;
            } else {
                Utils::print("Skipping disconnected scanned polygon while updating incremental map.");
            }
        }

        Polygon<T> knownBorder;
        std::vector<Polygon<T>> knownObstacles;
        if (!extractKnownComponent(tempStart->getPoint(), knownBorder, knownObstacles)) {
            Utils::print("No known free-space component contains the current robot position.");
            drawFailureState(iteration, "no_known_component");
            return false;
        }
        _mappedBorder = knownBorder;
        _mappedObstacles = knownObstacles;

        VisibilityGraph<T> iterationVG(
            this->_robot,
            knownBorder,
            knownObstacles,
            this->_resolution,
            false,
            this->_numThreads
        );
        bool startAdded = iterationVG.addVertex(tempStart);
        std::cout << "StartAdded: " << startAdded << std::endl;
        bool readStartAdded = iterationVG.addVertex(this->_start);
        std::cout << "ReadStartAdded: " << readStartAdded << std::endl;
        bool goalAdded = iterationVG.addVertex(this->_goal);
        _debugCurrentLocation = tempStart;
        _debugIterationBorder = knownBorder;
        _debugIterationObstacles = knownObstacles;
        _debugIterationGraph = iterationVG.getGraph();
        _debugStartAddedToIterationGraph = startAdded;
        _debugGoalAddedToIterationGraph = goalAdded;
        _debugDirectGoalConnection = startAdded && goalAdded &&
            graphHasEdge(_debugIterationGraph, tempStart, this->_goal);
        _graph = iterationVG.getGraph();
        _graph.setWeight(_alpha, _beta);

        if (goalAdded) {
            const auto shortestPath = _graph.shortestPath(
                tempStart,
                this->_goal,
                false
            );
            if (!shortestPath.empty()) {
                appendSegment(shortestPath);
                _explorationPath = fullExplorationPath;
                const auto realShortestPath = _graph.shortestPath(
                    this->_start,
                    this->_goal,
                    false
                );
                for (const auto &vertex : realShortestPath) {
                    const auto vertexPtr = std::make_shared<Vertex<T>>(vertex);
                    _shortestPath.push_back(vertexPtr);
                }
                drawCurrentIteration(iteration, this->_goal);
                return true;
            }
        }

        std::cout << "Iteration " << iteration + 1 << ": _alpha = " << _alpha << ", _beta = " << _beta << std::endl;
        tempGoal = calculateTemporaryGoal(tempStart);
        if (!tempGoal) {
            Utils::print("No temporary goal found in this iteration, stopping planning.");
            drawFailureState(iteration, "no_temporary_goal");
            return false;
        }
        std::cout << "Temporary goal: " << *tempGoal << std::endl;

        const auto shortestPath = _graph.shortestPath(
            tempStart,
            tempGoal,
            false
        );
        if (shortestPath.empty()) {
            Utils::print("No path found in this iteration, stopping planning.");
            drawFailureState(iteration, "no_path", tempGoal);
            return false;
        }

        appendSegment(shortestPath);
        _explorationPath = fullExplorationPath;
        drawCurrentIteration(iteration, tempGoal);
        _exploredVertices.insert(tempGoal);
        tempStart = tempGoal;
    }

    Utils::print("Incremental-mapping planning did not converge within the iteration limit.");
    drawFailureState(maxIterations - 1, "iteration_limit");
    return false;
}

template <typename T>
void DynamicRVG<T>::moverobotdebug(const Vertex<T> &nextLocation)
{
    const Vertex<T> previousLocation = _start ? *_start : Vertex<T>();
    this->_start = std::make_shared<Vertex<T>>(nextLocation);
    if (!this->_start->hasTheta() && previousLocation.hasTheta()) {
        this->_start->setTheta(previousLocation.getTheta());
    }
    _debugCurrentLocation = this->_start;
}
    
//TODO: We can have several visualization functions to help us debugging.
// For example, we can visualize the _visibleAreaInMap, the _explorationPath, the _shortestPath, and the merged visibility graph.
// We can also visualize the RVG built after every step
template <typename T>
void DynamicRVG<T>::drawVisibleArea(const std::string &name) const { // visualize the _visibleAreaInMap
    if (!_visibleAreaInMap.size()) {
        Utils::print("No visible area has been scanned.");
        return;
    }

    const std::filesystem::path outputDir = std::filesystem::path("build") / "dynamicrvgdebug";
    std::filesystem::create_directories(outputDir);
    const std::string suffix = name.empty() ? "" : "_" + name;
    const std::string imagePath = (outputDir / ("visible_area" + suffix + ".png")).string();
    const std::string scriptPath = (outputDir / ("drawVisibleArea" + suffix + ".py")).string();

    std::string pythonScript;
    pythonScript += "import matplotlib.pyplot as plt\n";
    pythonScript += "fig, ax = plt.subplots(dpi=500)\n";
    pythonScript += "ax.set_facecolor('gainsboro')\n";

    int polygonId = 0;
    auto appendPolygon = [&](const Polygon<T> &polygon,
                             const std::string &edgeColor,
                             const std::string &fillColor,
                             double alpha,
                             double lineWidth) {
        const std::string prefix = "poly_" + std::to_string(polygonId++);
        const auto x = polygon.getX();
        const auto y = polygon.getY();
        pythonScript += prefix + "_x = [";
        for (size_t i = 0; i < x.size(); ++i) {
            pythonScript += std::to_string(x[i]);
            if (i + 1 < x.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += prefix + "_y = [";
        for (size_t i = 0; i < y.size(); ++i) {
            pythonScript += std::to_string(y[i]);
            if (i + 1 < y.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        if (!fillColor.empty()) {
            pythonScript += "ax.fill(" + prefix + "_x, " + prefix + "_y, color='" + fillColor +
                            "', alpha=" + std::to_string(alpha) + ")\n";
        }
        pythonScript += "ax.plot(" + prefix + "_x, " + prefix + "_y, color='" + edgeColor +
                        "', linewidth=" + std::to_string(lineWidth) + ")\n";
    };

    for (const auto &obs : this->_obstacles) {
        appendPolygon(obs, "darkcyan", "lightcyan", 0.9, 1.0);
    }
    appendPolygon(_mappedBorder.size() ? _mappedBorder : _visibleAreaInMap, "goldenrod", "gold", 0.35, 1.5);
    for (const auto &hole : _mappedObstacles) {
        appendPolygon(hole, "goldenrod", "white", 1.0, 1.2);
    }

    const Vertex<T> &current = _debugCurrentLocation
        ? *_debugCurrentLocation
        : ((this->_start && this->_start->hasTheta()) ? *this->_start : this->_robot.getCentroid());
    const T robotTheta = current.hasTheta() ? current.getTheta() : static_cast<T>(0);
    const Polygon<T> robotFootprint = this->_robot.moveToCopy(
        current.getX(),
        current.getY(),
        robotTheta
    );
    appendPolygon(robotFootprint, "crimson", "mistyrose", 0.30, 1.2);
    pythonScript += "ax.plot([" + std::to_string(current.getX()) + "], [" + std::to_string(current.getY()) +
                    "], 'o', color='crimson', markersize=5)\n";

    pythonScript += "ax.set_aspect('equal', adjustable='box')\n";
    pythonScript += "plt.title('Visible area')\n";
    pythonScript += "plt.savefig('" + imagePath + "', dpi=500, bbox_inches='tight')\n";
    Utils::writeStringToFile<T>(pythonScript, scriptPath);
    Utils::print("Wrote", scriptPath);
}

template <typename T>
std::string DynamicRVG<T>::drawIteration(const std::string &name, const std::shared_ptr<Vertex<T>> &temporaryGoal) const
{
    const std::filesystem::path outputDir = std::filesystem::current_path() / "build" / "dynamicrvgdebug";
    std::filesystem::create_directories(outputDir);
    const std::string suffix = name.empty() ? "" : "_" + name;
    const std::string imagePath = (outputDir / ("iteration" + suffix + ".png")).string();
    const std::string scriptPath = (outputDir / ("drawIteration" + suffix + ".py")).string();

    const Vertex<T> &current = _debugCurrentLocation
        ? *_debugCurrentLocation
        : ((this->_start && this->_start->hasTheta()) ? *this->_start : this->_robot.getCentroid());
    const T currentTheta = current.hasTheta() ? current.getTheta() : static_cast<T>(0);
    const Polygon<T> currentRobotFootprint = this->_robot.moveToCopy(
        current.getX(),
        current.getY(),
        currentTheta
    );
    const size_t iterationVertexCount = _debugIterationGraph.getVertices().size();
    const size_t iterationEdgeCount = graphEdgeCount(_debugIterationGraph);
    const size_t globalVertexCount = _graph.getVertices().size();
    const size_t globalEdgeCount = graphEdgeCount(_graph);

    std::string pythonScript;
    pythonScript += "import matplotlib.pyplot as plt\n";
    pythonScript += "plt.rcParams['font.family'] = 'serif'\n";
    pythonScript += "plt.rcParams['font.serif'] = ['Times New Roman']\n";
    pythonScript += "fig, axs = plt.subplots(2, 2, figsize=(14, 12), dpi=350, constrained_layout=True)\n";
    pythonScript += "for axis in axs.flat:\n";
    pythonScript += "    axis.set_facecolor('gainsboro')\n";
    pythonScript += "    axis.set_aspect('equal', adjustable='box')\n";
    pythonScript += "    axis.grid(True, color='white', linewidth=0.5, alpha=0.35)\n";
    pythonScript += "    axis.tick_params(labelsize=7)\n";

    int polygonId = 0;
    auto appendPolygon = [&](const std::string &axisName,
                             const Polygon<T> &polygon,
                             const std::string &edgeColor,
                             const std::string &fillColor,
                             double alpha,
                             double lineWidth) {
        if (!polygon.size()) {
            return;
        }
        const std::string prefix = "poly_" + std::to_string(polygonId++);
        const auto x = polygon.getX();
        const auto y = polygon.getY();
        pythonScript += prefix + "_x = [";
        for (size_t i = 0; i < x.size(); ++i) {
            pythonScript += std::to_string(x[i]);
            if (i + 1 < x.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += prefix + "_y = [";
        for (size_t i = 0; i < y.size(); ++i) {
            pythonScript += std::to_string(y[i]);
            if (i + 1 < y.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        if (!fillColor.empty()) {
            pythonScript += axisName + ".fill(" + prefix + "_x, " + prefix + "_y, color='" + fillColor +
                            "', alpha=" + std::to_string(alpha) + ")\n";
        }
        pythonScript += axisName + ".plot(" + prefix + "_x, " + prefix + "_y, color='" + edgeColor +
                        "', linewidth=" + std::to_string(lineWidth) + ")\n";
    };

    const auto appendPoint = [&](const std::string &axisName,
                                 const Vertex<T> &vertex,
                                 const std::string &marker,
                                 const std::string &color,
                                 double markerSize) {
        pythonScript += axisName + ".plot([" + std::to_string(vertex.getX()) + "], [" +
                        std::to_string(vertex.getY()) + "], '" + marker + "', color='" + color +
                        "', markersize=" + std::to_string(markerSize) + ")\n";
    };
    const auto appendText = [&](const std::string &axisName,
                                const Vertex<T> &vertex,
                                const std::string &text,
                                const std::string &color) {
        pythonScript += axisName + ".text(" + std::to_string(vertex.getX()) + ", " +
                        std::to_string(vertex.getY()) + ", '" + text + "', color='" + color +
                        "', fontsize=8)\n";
    };
    const auto appendWorld = [&](const std::string &axisName) {
        appendPolygon(axisName, this->_border, "dimgray", "", 0.0, 1.0);
        for (const auto &obs : this->_obstacles) {
            appendPolygon(axisName, obs, "darkcyan", "lightcyan", 0.78, 1.0);
        }
    };
    const auto appendPath = [&](const std::string &axisName,
                                const std::vector<std::shared_ptr<Vertex<T>>> &path,
                                const std::string &color,
                                double lineWidth,
                                double markerSize) {
        if (path.empty()) {
            return;
        }
        pythonScript += "path_x = [";
        for (size_t i = 0; i < path.size(); ++i) {
            pythonScript += std::to_string(path[i]->getX());
            if (i + 1 < path.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += "path_y = [";
        for (size_t i = 0; i < path.size(); ++i) {
            pythonScript += std::to_string(path[i]->getY());
            if (i + 1 < path.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += axisName + ".plot(path_x, path_y, '-o', color='" + color +
                        "', linewidth=" + std::to_string(lineWidth) + ", markersize=" +
                        std::to_string(markerSize) + ")\n";
    };
    const auto appendRobotPose = [&](const std::string &axisName, const Polygon<T> &footprint) {
        appendPolygon(axisName, footprint, "crimson", "mistyrose", 0.28, 1.2);
        appendPoint(axisName, current, "o", "crimson", 5.0);
        appendText(axisName, current, " current", "crimson");
    };
    const auto appendGoalMarkers = [&](const std::string &axisName) {
        if (this->_goal) {
            appendPoint(axisName, *this->_goal, "*", "navy", 8.0);
            appendText(axisName, *this->_goal, " goal", "navy");
        }
        if (temporaryGoal) {
            appendPoint(axisName, *temporaryGoal, "x", "darkviolet", 8.0);
            appendText(axisName, *temporaryGoal, " temp", "darkviolet");
            if (temporaryGoal->hasTheta()) {
                const Polygon<T> tempGoalRobot = this->_robot.moveToCopy(
                    temporaryGoal->getX(),
                    temporaryGoal->getY(),
                    temporaryGoal->getTheta()
                );
                appendPolygon(axisName, tempGoalRobot, "darkviolet", "", 0.0, 1.0);
            }
        }
    };

    static const std::vector<std::string> scanEdgeColors = {
        "royalblue", "seagreen", "darkorange", "mediumorchid", "firebrick", "saddlebrown"
    };
    static const std::vector<std::string> scanFillColors = {
        "deepskyblue", "mediumspringgreen", "gold", "plum", "lightsalmon", "khaki"
    };

    appendWorld("axs[0, 0]");
    for (size_t i = 0; i < _debugVisibleAreas.size(); ++i) {
        const std::string edgeColor = scanEdgeColors[i % scanEdgeColors.size()];
        const std::string fillColor = scanFillColors[i % scanFillColors.size()];
        appendPolygon("axs[0, 0]", _debugVisibleAreas[i], edgeColor, fillColor, 0.18, 1.3);
    }
    for (size_t i = 0; i < _debugScanLocations.size(); ++i) {
        const std::string color = scanEdgeColors[i % scanEdgeColors.size()];
        appendPoint("axs[0, 0]", _debugScanLocations[i], "o", color, 4.0);
        appendText("axs[0, 0]", _debugScanLocations[i], " s" + std::to_string(i + 1), color);
    }
    appendRobotPose("axs[0, 0]", currentRobotFootprint);
    appendGoalMarkers("axs[0, 0]");
    pythonScript += "axs[0, 0].set_title('Raw visible areas (" + std::to_string(_debugVisibleAreas.size()) +
                    " scans)', fontsize=11)\n";

    appendWorld("axs[0, 1]");
    for (const auto &region : _debugMergedVisibleAreas) {
        appendPolygon("axs[0, 1]", Polygon<T>(region.outer_boundary()), "seagreen", "palegreen", 0.18, 1.4);
        for (auto holeIt = region.holes_begin(); holeIt != region.holes_end(); ++holeIt) {
            appendPolygon("axs[0, 1]", Polygon<T>(*holeIt), "seagreen", "white", 1.0, 1.2);
        }
    }
    appendPolygon("axs[0, 1]", _mappedBorder.size() ? _mappedBorder : _visibleAreaInMap, "goldenrod", "gold", 0.32, 2.0);
    for (const auto &hole : _mappedObstacles) {
        appendPolygon("axs[0, 1]", hole, "goldenrod", "white", 1.0, 1.4);
    }
    appendRobotPose("axs[0, 1]", currentRobotFootprint);
    appendGoalMarkers("axs[0, 1]");
    pythonScript += "axs[0, 1].set_title('Merged visible regions (" + std::to_string(_debugMergedVisibleAreas.size()) +
                    " components, selected highlighted)', fontsize=11)\n";

    appendWorld("axs[1, 0]");
    appendPolygon("axs[1, 0]", _debugIterationBorder, "royalblue", "lightskyblue", 0.12, 1.8);
    for (const auto &iterationObstacle : _debugIterationObstacles) {
        appendPolygon("axs[1, 0]", iterationObstacle, "royalblue", "white", 1.0, 1.4);
    }
    pythonScript += _debugIterationGraph.draw("axs[1, 0]");
    if (this->_goal) {
        const std::string directColor = _debugDirectGoalConnection ? "forestgreen" : "firebrick";
        pythonScript += "axs[1, 0].plot([" + std::to_string(current.getX()) + ", " +
                        std::to_string(this->_goal->getX()) + "], [" +
                        std::to_string(current.getY()) + ", " + std::to_string(this->_goal->getY()) +
                        "], '--', color='" + directColor + "', linewidth=1.2, alpha=0.85)\n";
    }
    appendRobotPose("axs[1, 0]", currentRobotFootprint);
    appendGoalMarkers("axs[1, 0]");
    pythonScript += "axs[1, 0].text(0.02, 0.98, 'start added: " +
                    std::string(_debugStartAddedToIterationGraph ? "yes" : "no") + "\\n"
                    "goal added: " + std::string(_debugGoalAddedToIterationGraph ? "yes" : "no") + "\\n"
                    "direct edge to goal: " + std::string(_debugDirectGoalConnection ? "yes" : "no") +
                    "', transform=axs[1, 0].transAxes, va='top', fontsize=8, "
                    "bbox=dict(facecolor='white', alpha=0.75, edgecolor='none'))\n";
    pythonScript += "axs[1, 0].set_title('Iteration RVG (V=" + std::to_string(iterationVertexCount) +
                    ", E=" + std::to_string(iterationEdgeCount) + ")', fontsize=11)\n";

    appendWorld("axs[1, 1]");
    pythonScript += _graph.draw("axs[1, 1]");
    appendPath("axs[1, 1]", _explorationPath, "navy", 2.0, 3.0);
    appendPath("axs[1, 1]", _shortestPath, "darkorange", 1.5, 2.0);
    appendRobotPose("axs[1, 1]", currentRobotFootprint);
    appendGoalMarkers("axs[1, 1]");
    pythonScript += "axs[1, 1].set_title('Accumulated RVG and paths (V=" + std::to_string(globalVertexCount) +
                    ", E=" + std::to_string(globalEdgeCount) + ")', fontsize=11)\n";

    pythonScript += "fig.suptitle('Dynamic RVG iteration debug', fontsize=14)\n";
    pythonScript += "plt.savefig('" + imagePath + "', dpi=500, bbox_inches='tight')\n";
    Utils::writeStringToFile<T>(pythonScript, scriptPath);
    Utils::print("Wrote", scriptPath);
    return scriptPath;
}

template <typename T>
std::string DynamicRVG<T>::drawFullPathAndEndGraph(const std::string &name) const
{
    const std::filesystem::path outputDir = std::filesystem::current_path();
    const std::string suffix = name.empty() ? "" : "_" + name;
    const std::string imagePath = (outputDir / ("fullPathAndEndGraph" + suffix + ".png")).string();
    const std::string scriptPath = (outputDir / ("drawFullPathAndEndGraph" + suffix + ".py")).string();
    std::vector<Vertex<T>> globalShortestPath;
    if (_start && _goal) {
        VisibilityGraph<T> globalVG(
            this->_robot,
            this->_border,
            this->_obstacles,
            this->_resolution,
            false,
            this->_numThreads
        );
        globalVG.setWeight(_alpha, _beta);
        globalShortestPath = globalVG.shortestPath(this->_start, this->_goal, 0, false);
    }

    std::string pythonScript;
    pythonScript += "import matplotlib.pyplot as plt\n";
    pythonScript += "fig, ax = plt.subplots(dpi=500)\n";
    pythonScript += "ax.set_facecolor('gainsboro')\n";

    int polygonId = 0;
    auto appendPolygon = [&](const Polygon<T> &polygon,
                             const std::string &edgeColor,
                             const std::string &fillColor,
                             double alpha,
                             double lineWidth) {
        if (!polygon.size()) {
            return;
        }
        const std::string prefix = "poly_" + std::to_string(polygonId++);
        const auto x = polygon.getX();
        const auto y = polygon.getY();
        pythonScript += prefix + "_x = [";
        for (size_t i = 0; i < x.size(); ++i) {
            pythonScript += std::to_string(x[i]);
            if (i + 1 < x.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += prefix + "_y = [";
        for (size_t i = 0; i < y.size(); ++i) {
            pythonScript += std::to_string(y[i]);
            if (i + 1 < y.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        if (!fillColor.empty()) {
            pythonScript += "ax.fill(" + prefix + "_x, " + prefix + "_y, color='" + fillColor +
                            "', alpha=" + std::to_string(alpha) + ")\n";
        }
        pythonScript += "ax.plot(" + prefix + "_x, " + prefix + "_y, color='" + edgeColor +
                        "', linewidth=" + std::to_string(lineWidth) + ")\n";
    };

    appendPolygon(this->_border, "dimgray", "", 0.0, 1.0);
    for (const auto &obs : this->_obstacles) {
        appendPolygon(obs, "darkcyan", "lightcyan", 0.8, 1.0);
    }

    pythonScript += _graph.draw();

    const auto appendRobotFootprint = [&](const Vertex<T> &pose,
                                          const std::string &edgeColor,
                                          const std::string &fillColor,
                                          double alpha,
                                          double lineWidth) {
        if (!pose.hasTheta()) {
            return;
        }
        const Polygon<T> robotFootprint = this->_robot.moveToCopy(
            pose.getX(),
            pose.getY(),
            pose.getTheta()
        );
        appendPolygon(robotFootprint, edgeColor, fillColor, alpha, lineWidth);
    };

    const auto normalizeAngle = [](T angle) {
        while (angle <= -static_cast<T>(PI)) {
            angle += static_cast<T>(2 * PI);
        }
        while (angle > static_cast<T>(PI)) {
            angle -= static_cast<T>(2 * PI);
        }
        return angle;
    };

    const auto interpolatePath = [&](const std::vector<Vertex<T>> &path) {
        std::vector<Vertex<T>> interpolatedPath;
        if (path.empty()) {
            return interpolatedPath;
        }

        interpolatedPath.push_back(path.front());
        const int samplesPerEdge = 5;

        for (size_t i = 1; i < path.size(); ++i) {
            const Vertex<T> &from = path[i - 1];
            const Vertex<T> &to = path[i];
            const bool hasTheta = from.hasTheta() && to.hasTheta();
            const T deltaTheta = hasTheta ? normalizeAngle(to.getTheta() - from.getTheta()) : static_cast<T>(0);
            const int numSamples = samplesPerEdge;

            for (int sample = 1; sample <= numSamples; ++sample) {
                const T t = static_cast<T>(sample) / static_cast<T>(numSamples);
                Vertex<T> pose(
                    from.getX() + (to.getX() - from.getX()) * t,
                    from.getY() + (to.getY() - from.getY()) * t
                );
                if (hasTheta) {
                    pose.setTheta(from.getTheta() + deltaTheta * t);
                }
                interpolatedPath.push_back(pose);
            }
        }

        return interpolatedPath;
    };

    const auto pathCost = [&](const std::vector<Vertex<T>> &path) {
        T cost = 0;
        for (size_t i = 1; i < path.size(); ++i) {
            cost += _alpha * path[i - 1].dist(path[i]);
            if (path[i - 1].hasTheta() && path[i].hasTheta()) {
                cost += _beta * path[i - 1].rotationalDist(path[i]);
            }
        }
        return cost;
    };

    const auto appendPath = [&](const std::string &prefix,
                                const std::vector<std::shared_ptr<Vertex<T>>> &path,
                                const std::string &style,
                                const std::string &label) {
        if (path.empty()) {
            return;
        }
        pythonScript += prefix + "_x = [";
        for (size_t i = 0; i < path.size(); ++i) {
            pythonScript += std::to_string(path[i]->getX());
            if (i + 1 < path.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += prefix + "_y = [";
        for (size_t i = 0; i < path.size(); ++i) {
            pythonScript += std::to_string(path[i]->getY());
            if (i + 1 < path.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += "ax.plot(" + prefix + "_x, " + prefix + "_y, " + style + ", label='" + label + "')\n";
    };

    const auto appendValuePath = [&](const std::string &prefix,
                                     const std::vector<Vertex<T>> &path,
                                     const std::string &style,
                                     const std::string &label) {
        if (path.empty()) {
            return;
        }
        pythonScript += prefix + "_x = [";
        for (size_t i = 0; i < path.size(); ++i) {
            pythonScript += std::to_string(path[i].getX());
            if (i + 1 < path.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += prefix + "_y = [";
        for (size_t i = 0; i < path.size(); ++i) {
            pythonScript += std::to_string(path[i].getY());
            if (i + 1 < path.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += "ax.plot(" + prefix + "_x, " + prefix + "_y, " + style + ", label='" + label + "')\n";
    };

    std::vector<Vertex<T>> explorationPathValues;
    explorationPathValues.reserve(_explorationPath.size());
    for (const auto &vertex : _explorationPath) {
        explorationPathValues.push_back(*vertex);
    }

    std::vector<Vertex<T>> dynamicShortestPathValues;
    dynamicShortestPathValues.reserve(_shortestPath.size());
    for (const auto &vertex : _shortestPath) {
        dynamicShortestPathValues.push_back(*vertex);
    }

    appendPath(
        "exploration_path",
        _explorationPath,
        "'-o', color='navy', linewidth=1.2, markersize=2.0",
        "Exploration path (cost=" + std::to_string(pathCost(explorationPathValues)) + ")"
    );
    appendPath(
        "dynamic_shortest_path",
        _shortestPath,
        "'--s', color='darkorange', linewidth=1.0, markersize=1.8",
        "DynamicRVG shortest path (cost=" + std::to_string(pathCost(dynamicShortestPathValues)) + ")"
    );
    appendValuePath(
        "global_shortest_path",
        globalShortestPath,
        "'-', color='forestgreen', linewidth=1.2",
        "Global RVG shortest path (cost=" + std::to_string(pathCost(globalShortestPath)) + ")"
    );

    std::vector<Vertex<T>> finalPathForRobot;
    if (!_shortestPath.empty()) {
        finalPathForRobot.reserve(_shortestPath.size());
        for (const auto &vertex : _shortestPath) {
            finalPathForRobot.push_back(*vertex);
        }
    } else {
        finalPathForRobot = globalShortestPath;
    }
    const std::vector<Vertex<T>> interpolatedRobotPath = interpolatePath(finalPathForRobot);
    for (size_t i = 0; i < interpolatedRobotPath.size(); ++i) {
        const bool isEndpoint = (i == 0) || (i + 1 == interpolatedRobotPath.size());
        appendRobotFootprint(
            interpolatedRobotPath[i],
            "mediumpurple",
            "plum",
            isEndpoint ? 0.18 : 0.10,
            isEndpoint ? 1.0 : 0.6
        );
    }

    if (!_explorationPath.empty()) {
        pythonScript += "ax.plot([" + std::to_string(_explorationPath.front()->getX()) + "], [" +
                        std::to_string(_explorationPath.front()->getY()) +
                        "], 'o', color='crimson', markersize=5)\n";
        pythonScript += "ax.text(" + std::to_string(_explorationPath.front()->getX()) + ", " +
                        std::to_string(_explorationPath.front()->getY()) +
                        ", ' start', color='crimson', fontsize=8)\n";
    }

    if (this->_goal && this->_goal->hasTheta()) {
        pythonScript += "ax.plot([" + std::to_string(this->_goal->getX()) + "], [" +
                        std::to_string(this->_goal->getY()) +
                        "], '*', color='navy', markersize=8)\n";
        pythonScript += "ax.text(" + std::to_string(this->_goal->getX()) + ", " +
                        std::to_string(this->_goal->getY()) +
                        ", ' goal', color='navy', fontsize=8)\n";
    }

    if (_start) {
        appendRobotFootprint(*_start, "crimson", "mistyrose", 0.35, 1.2);
    }
    if (_goal) {
        appendRobotFootprint(*_goal, "forestgreen", "honeydew", 0.30, 1.2);
    }

    pythonScript += "ax.set_aspect('equal', adjustable='box')\n";
    pythonScript += "ax.legend(loc='best', fontsize=8)\n";
    pythonScript += "plt.title('Dynamic RVG full path and final graph')\n";
    pythonScript += "plt.savefig('" + imagePath + "', dpi=500, bbox_inches='tight')\n";
    Utils::writeStringToFile<T>(pythonScript, scriptPath);
    Utils::print("Wrote", scriptPath);
    return scriptPath;
}

template<typename T>
const std::vector<std::shared_ptr<Vertex<T>>>& DynamicRVG<T>::getExplorationPath() const{
    return _explorationPath;
}

template<typename T>
const std::vector<std::shared_ptr<Vertex<T>>>& DynamicRVG<T>::getShortestPath() const{
    return _shortestPath;
}

template<typename T>
const Graph<T> & DynamicRVG<T>::getGraph() const{
    return _graph;
}

template<typename T>
void DynamicRVG<T>::setGoal(const std::shared_ptr<Vertex<T>> &goal){
    _goal = goal;    
}

template<typename T>
void DynamicRVG<T>::setGraph(const Graph<T> &graph){
    _graph = graph;
}

template<typename T>
void DynamicRVG<T>::setWeight(T alpha, T beta){
    _alpha = alpha;
    _beta = beta;
}

template<typename T>
void DynamicRVG<T>::setWriteIterationVisualizations(bool enabled){
    _writeIterationVisualizations = enabled;
}

template<typename T>
void DynamicRVG<T>::setIterationVisualizationTag(const std::string &tag){
    _iterationVisualizationTag = tag;
}

template class DynamicRVG<double>;
template class DynamicRVG<float>;
}
