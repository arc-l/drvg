#include "DynamicRVG.h"
#include <VisibilityGraph/Utils.h>
#include <filesystem>
#include <Utils/Utils.h>

namespace RotationalVisibilityGraph {
    
template <typename T>
const Polygon<T> &DynamicRVG<T>::scanVisibleArea(const Vertex<T> & currentLocation) // scan the environment at the current robot location and then update the _visibleAreaInMap
{
    std::vector<Polygon_2> map;
    map.reserve(this->_obstacles.size() + 1);
    for (const auto &obs : this->_obstacles) map.push_back(obs.getPolygon());
    map.push_back(this->_border.getPolygon());
    Arrangement_2 env = obsToArrangement<T>(map), visibleArea;
    PL_2 pointLocation(env);
    auto obj = pointLocation.locate(currentLocation.getPoint());
    auto face = boost::get<Arrangement_2::Face_const_handle>(&obj);
    if (!face) {
        _visibleAreaInMap = Polygon<T>();
        return _visibleAreaInMap;
    }
    VQ visibility(env);
    Face_handle visibleFace = visibility.compute_visibility(currentLocation.getPoint(), *face, visibleArea);
    std::vector<Vertex<T>> boundary;
    auto edge = visibleFace->outer_ccb();
    do {
        const auto &point = edge->source()->point();
        boundary.emplace_back(CGAL::to_double(point.x()), CGAL::to_double(point.y()));
        ++edge;
    } while (edge != visibleFace->outer_ccb());
    if (boundary.size() < 3) {
        _visibleAreaInMap = Polygon<T>();
        return _visibleAreaInMap;
    }
    _visibleAreaInMap = Polygon<T>(boundary, false);
    return _visibleAreaInMap;
}

template <typename T>
void DynamicRVG<T>::calculateTemporaryGoal()
{
    _hasTemporaryGoal = false;
    _temporaryGoal = Vertex<T>();

    if (_visibleAreaInMap.size()) {
        const Point_2 goalPoint = this->_goal.getPoint();
        if (IN_POLYGON(goalPoint, _visibleAreaInMap.getPolygon()) ||
            ON_EDGE(goalPoint, _visibleAreaInMap.getPolygon())) {
            _temporaryGoal = this->_goal;
            _hasTemporaryGoal = true;
            return;
        }
    }

    const auto &vertices = _graph.getVertices();
    if (vertices.empty()) {
        return;
    }

    std::shared_ptr<Vertex<T>> closestVertex = nullptr;
    T closestDistance = std::numeric_limits<T>::max();
    for (const auto &vertex : vertices) {
        if (_exploredVertices.find(vertex) != _exploredVertices.end()) {
            continue;
        }
        const T distanceToGoal = vertex->dist(this->_goal);
        if (distanceToGoal < closestDistance) {
            closestDistance = distanceToGoal;
            closestVertex = vertex;
        }
    }

    if (closestVertex) {
        _temporaryGoal = *closestVertex;
        _hasTemporaryGoal = true;
    }
}

template <typename T>
VisibilityGraph<T> DynamicRVG<T>::buildVisibilityGraph()
{
    VisibilityGraph<T> visibilityGraph(
        this->_robot,
        this->_visibleAreaInMap,
        {},
        this->_resolution,
        false,
        this->_numThreads
    );

    bool startAdded = visibilityGraph.addVertex(std::make_shared<Vertex<T>>(this->_start));
    std::cout << "Start vertex " << this->_start << (startAdded ? " added to " : " not added to ") << "visibility graph." << std::endl;
    if (_hasTemporaryGoal) {
        bool goalAdded = visibilityGraph.addVertex(std::make_shared<Vertex<T>>(_temporaryGoal));
        std::cout << "Temporary goal vertex " << _temporaryGoal << (goalAdded ? " added to " : " not added to ") << "visibility graph." << std::endl;
    }
    return visibilityGraph;
}

template <typename T>
void DynamicRVG<T>::calculateShortestPath()
{
    _explorationPath.clear();
    _shortestPath.clear();

    if (!_hasTemporaryGoal) {
        return;
    }

    const auto appendPath = [&](const std::vector<Vertex<T>> &path) {
        for (const auto &vertex : path) {
            const auto vertexPtr = std::make_shared<Vertex<T>>(vertex);
            _explorationPath.push_back(vertexPtr);
            _shortestPath.push_back(vertexPtr);
        }
    };

    const Point_2 goalPoint = this->_goal.getPoint();
    const bool goalVisible = _visibleAreaInMap.size() &&
        (IN_POLYGON(goalPoint, _visibleAreaInMap.getPolygon()) ||
         ON_EDGE(goalPoint, _visibleAreaInMap.getPolygon()));
    if (goalVisible && this->_goal.dist(_temporaryGoal) < 1e-5) {
        appendPath(std::vector<Vertex<T>>{this->_start, this->_goal});
        return;
    }

    const std::shared_ptr<Vertex<T>> start = std::make_shared<Vertex<T>>(this->_start);
    const std::shared_ptr<Vertex<T>> goal = std::make_shared<Vertex<T>>(_temporaryGoal);
    const std::vector<Vertex<T>> path = _graph.shortestPath(start, goal, false);
    appendPath(path);
    if (_explorationPath.empty()) {
        Utils::print("No path found from start to temporary goal");
    }
}

template <typename T>
bool DynamicRVG<T>::plan(const Vertex<T> &start, const Vertex<T> &goal)
{
    this->_start = start;
    this->_goal = goal;
    //TODO: No need to set default values for the start and the goal. If the user didn't provide these values, just throw an error.
    if (!this->_start.hasTheta()) {
        this->_start.setBounds(0, 0);
        this->_start.setTheta(0);
    }
    if (!this->_goal.hasTheta()) {
        this->_goal.setBounds(this->_start.getThetaLb(), this->_start.getThetaUb());
        this->_goal.setTheta(this->_start.getTheta());
    }

    _visibleAreaInMap = Polygon<T>();
    _temporaryGoal = Vertex<T>();
    _hasTemporaryGoal = false;
    _graph = Graph<T>();
    _exploredVertices.clear();
    _explorationPath.clear();
    _shortestPath.clear();

    std::vector<std::shared_ptr<Vertex<T>>> fullExplorationPath;
    const auto appendSegment = [&](const std::vector<std::shared_ptr<Vertex<T>>> &segment) {
        for (size_t i = 0; i < segment.size(); ++i) {
            if (!fullExplorationPath.empty() && i == 0 && *fullExplorationPath.back() == *segment[i]) {
                continue;
            }
            fullExplorationPath.push_back(std::make_shared<Vertex<T>>(*segment[i]));
        }
    };
    size_t failureDrawCounter = 0;
    const auto drawFailureState = [&](int iteration, const std::string &reason) {
        _explorationPath = fullExplorationPath;
        _shortestPath = fullExplorationPath;
        ++failureDrawCounter;
        std::string pathToScript = "failed_" + std::to_string(failureDrawCounter) + "_iter_" + std::to_string(iteration + 1) + "_" + reason;
        drawIteration(pathToScript);
        RotationalVisibilityGraph::Utils::runPythonScriptAndRemove<T>(pathToScript);
    };

    const int maxIterations = std::max(1, static_cast<int>(this->_obstacles.size()) * 4 + 10);
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        scanVisibleArea(_start);
        if (this->_start.dist(this->_goal) < 1e-5) {
            if (fullExplorationPath.empty()) {
                fullExplorationPath.push_back(std::make_shared<Vertex<T>>(this->_start));
            }
            _explorationPath = fullExplorationPath;
            _shortestPath = fullExplorationPath;
            return true;
        }

        VisibilityGraph<T> iterationVG = buildVisibilityGraph();
        _graph = iterationVG.getGraph();
        calculateTemporaryGoal();
        if (!_hasTemporaryGoal) {
            Utils::print("No temporary goal found in this iteration, stopping planning.");
            drawFailureState(iteration, "no_temporary_goal");
            return false;
        }

        // iterationVG = buildVisibilityGraph();
        // _graph = iterationVG.getGraph();
        // calculateShortestPath();
        iterationVG._addStartAndGoal(std::make_shared<Vertex<T>>(this->_start), std::make_shared<Vertex<T>>(_temporaryGoal));
        const auto shortestPath = iterationVG.shortestPath(
            std::make_shared<Vertex<T>>(this->_start),
            std::make_shared<Vertex<T>>(_temporaryGoal)
        );
        _shortestPath.clear();
        for(const auto& vertex: shortestPath) {
            _shortestPath.push_back(std::make_shared<Vertex<T>>(vertex));
        }
        if (_shortestPath.empty()) {
            Utils::print("No path found in this iteration, stopping planning.");
            drawFailureState(iteration, "no_path");
            return false;
        }

        appendSegment(_shortestPath);
        _explorationPath = fullExplorationPath;

        if (this->_goal.dist(_temporaryGoal) < 1e-5) {
            _shortestPath = _explorationPath;
            return true;
        }

        _exploredVertices.insert(std::make_shared<Vertex<T>>(_temporaryGoal));
        moverobotdebug(_temporaryGoal);
    }

    Utils::print("Planning did not converge within the iteration limit.");
    drawFailureState(maxIterations - 1, "iteration_limit");
    return false;
} // plan a path from start to goal using the visibility graph

template <typename T>
void DynamicRVG<T>::moverobotdebug(const Vertex<T> &nextLocation)
{
    const Vertex<T> previousLocation = this->_start;
    this->_start = nextLocation;
    if (!this->_start.hasTheta() && previousLocation.hasTheta()) {
        this->_start.setTheta(previousLocation.getTheta());
    }
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
    appendPolygon(_visibleAreaInMap, "goldenrod", "gold", 0.35, 1.5);

    const Vertex<T> &current = this->_start.hasTheta() ? this->_start : this->_robot.getCentroid();
    pythonScript += "ax.plot([" + std::to_string(current.getX()) + "], [" + std::to_string(current.getY()) +
                    "], 'o', color='crimson', markersize=5)\n";

    pythonScript += "ax.set_aspect('equal', adjustable='box')\n";
    pythonScript += "plt.title('Visible area')\n";
    pythonScript += "plt.savefig('" + imagePath + "', dpi=500, bbox_inches='tight')\n";
    Utils::writeStringToFile<T>(pythonScript, scriptPath);
    Utils::print("Wrote", scriptPath);
}

template <typename T>
void DynamicRVG<T>::drawIteration(const std::string &name) const
{
    const std::filesystem::path outputDir = std::filesystem::path("build") / "dynamicrvgdebug";
    std::filesystem::create_directories(outputDir);
    const std::string suffix = name.empty() ? "" : "_" + name;
    const std::string imagePath = (outputDir / ("iteration" + suffix + ".png")).string();
    const std::string scriptPath = (outputDir / ("drawIteration" + suffix + ".py")).string();

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
    appendPolygon(_visibleAreaInMap, "goldenrod", "gold", 0.35, 1.5);

    pythonScript += _graph.draw();

    if (!_explorationPath.empty()) {
        pythonScript += "path_x = [";
        for (size_t i = 0; i < _explorationPath.size(); ++i) {
            pythonScript += std::to_string(_explorationPath[i]->getX());
            if (i + 1 < _explorationPath.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += "path_y = [";
        for (size_t i = 0; i < _explorationPath.size(); ++i) {
            pythonScript += std::to_string(_explorationPath[i]->getY());
            if (i + 1 < _explorationPath.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += "ax.plot(path_x, path_y, '-o', color='navy', linewidth=2.0, markersize=3)\n";
    }

    const Vertex<T> &start = this->_start.hasTheta() ? this->_start : this->_robot.getCentroid();
    pythonScript += "ax.plot([" + std::to_string(start.getX()) + "], [" + std::to_string(start.getY()) +
                    "], 'o', color='crimson', markersize=5)\n";
    pythonScript += "ax.text(" + std::to_string(start.getX()) + ", " + std::to_string(start.getY()) +
                    ", ' start', color='crimson', fontsize=8)\n";

    if (this->_goal.hasTheta()) {
        pythonScript += "ax.plot([" + std::to_string(this->_goal.getX()) + "], [" +
                        std::to_string(this->_goal.getY()) +
                        "], '*', color='navy', markersize=8)\n";
        pythonScript += "ax.text(" + std::to_string(this->_goal.getX()) + ", " +
                        std::to_string(this->_goal.getY()) +
                        ", ' goal', color='navy', fontsize=8)\n";
    }

    if (_hasTemporaryGoal) {
        pythonScript += "ax.plot([" + std::to_string(_temporaryGoal.getX()) + "], [" +
                        std::to_string(_temporaryGoal.getY()) +
                        "], 'x', color='darkviolet', markersize=8, markeredgewidth=2)\n";
        pythonScript += "ax.text(" + std::to_string(_temporaryGoal.getX()) + ", " +
                        std::to_string(_temporaryGoal.getY()) +
                        ", ' temp goal', color='darkviolet', fontsize=8)\n";
    }

    pythonScript += "ax.set_aspect('equal', adjustable='box')\n";
    pythonScript += "plt.title('Dynamic RVG iteration')\n";
    pythonScript += "plt.savefig('" + imagePath + "', dpi=500, bbox_inches='tight')\n";
    Utils::writeStringToFile<T>(pythonScript, scriptPath);
    Utils::print("Wrote", scriptPath);
}

template <typename T>
void DynamicRVG<T>::drawFullPathAndEndGraph(const std::string &name) const
{
    const std::filesystem::path outputDir = std::filesystem::path("build") / "dynamicrvgdebug";
    std::filesystem::create_directories(outputDir);
    const std::string suffix = name.empty() ? "" : "_" + name;
    const std::string imagePath = (outputDir / ("fullPathAndEndGraph" + suffix + ".png")).string();
    const std::string scriptPath = (outputDir / ("drawFullPathAndEndGraph" + suffix + ".py")).string();

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

    pythonScript += _graph.draw();

    if (!_explorationPath.empty()) {
        pythonScript += "path_x = [";
        for (size_t i = 0; i < _explorationPath.size(); ++i) {
            pythonScript += std::to_string(_explorationPath[i]->getX());
            if (i + 1 < _explorationPath.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += "path_y = [";
        for (size_t i = 0; i < _explorationPath.size(); ++i) {
            pythonScript += std::to_string(_explorationPath[i]->getY());
            if (i + 1 < _explorationPath.size()) pythonScript += ", ";
        }
        pythonScript += "]\n";
        pythonScript += "ax.plot(path_x, path_y, '-o', color='navy', linewidth=2.0, markersize=3)\n";
    }

    if (!_explorationPath.empty()) {
        pythonScript += "ax.plot([" + std::to_string(_explorationPath.front()->getX()) + "], [" +
                        std::to_string(_explorationPath.front()->getY()) +
                        "], 'o', color='crimson', markersize=5)\n";
        pythonScript += "ax.text(" + std::to_string(_explorationPath.front()->getX()) + ", " +
                        std::to_string(_explorationPath.front()->getY()) +
                        ", ' start', color='crimson', fontsize=8)\n";
    }

    if (this->_goal.hasTheta()) {
        pythonScript += "ax.plot([" + std::to_string(this->_goal.getX()) + "], [" +
                        std::to_string(this->_goal.getY()) +
                        "], '*', color='navy', markersize=8)\n";
        pythonScript += "ax.text(" + std::to_string(this->_goal.getX()) + ", " +
                        std::to_string(this->_goal.getY()) +
                        ", ' goal', color='navy', fontsize=8)\n";
    }

    pythonScript += "ax.set_aspect('equal', adjustable='box')\n";
    pythonScript += "plt.title('Dynamic RVG full path and final graph')\n";
    pythonScript += "plt.savefig('" + imagePath + "', dpi=500, bbox_inches='tight')\n";
    Utils::writeStringToFile<T>(pythonScript, scriptPath);
    Utils::print("Wrote", scriptPath);
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
void DynamicRVG<T>::setGoal(const Vertex<T> &goal){
    _goal = goal;    
}

template<typename T>
void DynamicRVG<T>::setGraph(const Graph<T> &graph){
    _graph = graph;
}

template class DynamicRVG<double>;
template class DynamicRVG<float>;
}
