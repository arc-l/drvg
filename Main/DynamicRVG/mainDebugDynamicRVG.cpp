#include <DynamicVisibilityGraph/DynamicRVG.h>
#include <VisibilityGraph/MapGenerator.h>
#include <VisibilityGraph/RectangleObstacleMapGenerator.h>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

// Uncomment to generate random DynamicRVG full-path scripts in build/dynamicrvgdebug.
#define WRITE_DYNAMIC_RVG_RANDOM_SCENES

using namespace RotationalVisibilityGraph;

typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

Polygon<T> makeRobot() {
  return Polygon<T>({
      Vertex<T>(-3, 0),
      Vertex<T>(-2, 0),
      Vertex<T>(-2, 2),
      Vertex<T>(-3, 2)
  }, false);
}

void expect(bool ok, const std::string &name) {
  Utils::print(ok ? "[PASS]" : "[FAIL]", name);
  if (!ok) throw std::runtime_error(name);
}

int main() {
  constexpr int resolution = 36;
  constexpr int numThreads = 16;
  constexpr int targetSuccessfulScenes = 3;
  constexpr int maxAttempts = 20;
  constexpr int mapSize = 30;
  constexpr int numberOfObstacles = 8;
  constexpr T minObstacleSize = 2.0;
  constexpr T maxObstacleSize = 4.0;

  const Polygon<T> robot = makeRobot();
  const std::filesystem::path outputDir = std::filesystem::path("build") / "dynamicrvgdebug";
  std::filesystem::create_directories(outputDir);

#ifdef WRITE_DYNAMIC_RVG_RANDOM_SCENES
  int successes = 0;
  for (int attempt = 1; attempt <= maxAttempts && successes < targetSuccessfulScenes; ++attempt) {
    RectangleObstacleMapGenerator<T> mapGenerator(mapSize, numberOfObstacles, minObstacleSize, maxObstacleSize, robot);
    mapGenerator.generateMap();

    const Vertex<T> start = mapGenerator.getStart();
    const Vertex<T> goal = mapGenerator.getGoal();
    DynamicRVG<T> dynamicRVG(robot, mapGenerator.getBorder(), mapGenerator.getObstacles(), resolution, numThreads);
    const bool planned = dynamicRVG.plan(start, goal);
    if (!planned) {
      Utils::print("Skipping random attempt", attempt, "because plan() failed.");
      continue;
    }

    ++successes;
    const std::string sceneName = "random_scene_" + std::to_string(successes);
    dynamicRVG.drawFullPathAndEndGraph(sceneName);

    const auto scriptPath = outputDir / ("drawFullPathAndEndGraph_" + sceneName + ".py");
    expect(std::filesystem::exists(scriptPath), "drawFullPathAndEndGraph writes script for " + sceneName);
    expect(!dynamicRVG._explorationPath.empty(), "plan populates exploration path for " + sceneName);
    expect(dynamicRVG._graph.size() > 0, "plan leaves final graph for " + sceneName);
  }

  expect(successes == targetSuccessfulScenes, "random DynamicRVG generation produced enough successful scenes");
#else
  Utils::print("Random DynamicRVG generation disabled. Uncomment WRITE_DYNAMIC_RVG_RANDOM_SCENES in mainDebugDynamicRVG.cpp to enable it.");
#endif

  return 0;
}
