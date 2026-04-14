#include <DynamicVisibilityGraph/DynamicRVG.h>
#include <Utils/Pragma.h>
#include <Utils/Utils.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Utils.h>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

using namespace RotationalVisibilityGraph;
using T = double;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

int main(int argc, char *argv[]) {
  if (argc < 4) {
    throw std::runtime_error(
        "Usage: mainDebugDynamicRVG <config.xml> <resolution> <numThreads> [figPath] [plan|incremental]"
    );
  }

  tinyxml2::XMLDocument pt;
  if (pt.LoadFile(argv[1]) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to load config file");
  }

  const int resolution = std::stoi(argv[2]);
  const int numThreads = std::stoi(argv[3]);
  const std::string figPath = argc > 4 ? argv[4] : "";
  const std::string plannerMode = argc > 5 ? argv[5] : "plan";

  const Polygon<T> robot = RotationalVisibilityGraph::Utils::getRobot<T>(pt);
  const std::vector<Polygon<T>> obstacles = RotationalVisibilityGraph::Utils::getObstacles<T>(pt);

  Polygon<T> map;
  const bool useBoundary = RotationalVisibilityGraph::Utils::get<bool>(
      *pt.RootElement()->FirstChildElement("environment"),
      "useBoundary",
      false
  );
  if (useBoundary) {
    map = RotationalVisibilityGraph::Utils::getBoundary<T>(pt);
  } else {
    const int mapSize = RotationalVisibilityGraph::Utils::get<int>(
        *pt.RootElement()->FirstChildElement("environment"),
        "mapSize",
        100
    );
    map = RotationalVisibilityGraph::Utils::getMap<T>(mapSize);
  }

  const auto plannerSettings = pt.RootElement()->FirstChildElement("plannerSettings");
  const std::shared_ptr<Vertex<T>> start = RotationalVisibilityGraph::Utils::getVertex<T>(
      *plannerSettings->FirstChildElement("start")->FirstChildElement("Vertex")
  );
  const std::shared_ptr<Vertex<T>> goal = RotationalVisibilityGraph::Utils::getVertex<T>(
      *plannerSettings->FirstChildElement("goal")->FirstChildElement("Vertex")
  );

  RotationalVisibilityGraph::Utils::print("Setup:",
                                          "Start", *start,
                                          "Goal", *goal,
                                          "Resolution", resolution,
                                          "NumThreads", numThreads,
                                          "Draw Graph", figPath,
                                          "Planner Mode", plannerMode);

  if (plannerMode != "plan" && plannerMode != "incremental") {
    throw std::runtime_error("Planner mode must be either 'plan' or 'incremental'");
  }

  DynamicRVG<T> dynamicRVG(robot, map, obstacles, resolution, numThreads);
  dynamicRVG.setWeight(1.0, 0.1);

  const bool planned = plannerMode == "incremental"
                           ? dynamicRVG.planIncrementalMapping(start, goal)
                           : dynamicRVG.plan(start, goal);
  RotationalVisibilityGraph::Utils::print("DynamicRVG planned:", planned);
  RotationalVisibilityGraph::Utils::print("Exploration path vertices:", dynamicRVG.getExplorationPath().size());
  RotationalVisibilityGraph::Utils::print("Final graph size:", dynamicRVG.getGraph().size());

  if (!planned) {
    return 1;
  }

  if (!figPath.empty()) {
    const std::string outputName = std::filesystem::path(figPath).stem().string();
    const std::string scriptPath = dynamicRVG.drawFullPathAndEndGraph(outputName);
    RotationalVisibilityGraph::Utils::runPythonScriptAndRemove<T>(scriptPath);
  }

  return 0;
}
