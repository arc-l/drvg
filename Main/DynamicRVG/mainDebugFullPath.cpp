#include <DynamicVisibilityGraph/DynamicRVG.h>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

// Uncomment to generate full-path scripts in build/dynamicrvgdebug.
#define WRITE_DYNAMIC_RVG_FULL_PATH

using namespace RotationalVisibilityGraph;

typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

struct Scene {
  std::string name;
  Polygon<T> robot, map;
  std::vector<Polygon<T>> obstacles;
  Vertex<T> start, goal;
};

Polygon<T> makeMap(T mapSize) {
  return Polygon<T>({
      Vertex<T>(-mapSize / 2., -mapSize / 2.),
      Vertex<T>( mapSize / 2., -mapSize / 2.),
      Vertex<T>( mapSize / 2.,  mapSize / 2.),
      Vertex<T>(-mapSize / 2.,  mapSize / 2.)
  }, true);
}

Polygon<T> makeRobot() {
  return Polygon<T>({
      Vertex<T>(-3, 0),
      Vertex<T>(-2, 0),
      Vertex<T>(-2, 2),
      Vertex<T>(-3, 2)
  }, false);
}

std::vector<Scene> makeScenes() {
  const Polygon<T> robot = makeRobot();
  const Polygon<T> map = makeMap(20.);

  Vertex<T> start = robot.getCentroid();
  start.setBounds(0, 0);
  start.setTheta(0);

  Vertex<T> goal1(7, 0);
  goal1.setBounds(0, 0);
  goal1.setTheta(0);

  Vertex<T> goal2(7, 4);
  goal2.setBounds(0, 0);
  goal2.setTheta(0);

  Vertex<T> goal3(8, -3);
  goal3.setBounds(0, 0);
  goal3.setTheta(0);

  return {
      {
          "narrow_passage_full_path",
          robot,
          map,
          {
              Polygon<T>({Vertex<T>(2, -1), Vertex<T>(4, -1), Vertex<T>(4, -2), Vertex<T>(2, -2)}, true),
              Polygon<T>({Vertex<T>(2,  1), Vertex<T>(4,  1), Vertex<T>(4,  2), Vertex<T>(2,  2)}, true)
          },
          start,
          goal1
      },
      {
          "center_block_full_path",
          robot,
          map,
          {
              Polygon<T>({Vertex<T>(1, -2), Vertex<T>(4, -2), Vertex<T>(4, 2), Vertex<T>(1, 2)}, true)
          },
          start,
          goal2
      },
      {
          "zigzag_full_path",
          robot,
          map,
          {
              Polygon<T>({Vertex<T>(0, -4), Vertex<T>(2, -4), Vertex<T>(2, -1), Vertex<T>(0, -1)}, true),
              Polygon<T>({Vertex<T>(3,  1), Vertex<T>(5,  1), Vertex<T>(5,  4), Vertex<T>(3,  4)}, true),
              Polygon<T>({Vertex<T>(6, -4), Vertex<T>(8, -4), Vertex<T>(8,  0), Vertex<T>(6,  0)}, true)
          },
          start,
          goal3
      }
  };
}

void expect(bool ok, const std::string &name) {
  Utils::print(ok ? "[PASS]" : "[FAIL]", name);
  if (!ok) throw std::runtime_error(name);
}

int main() {
  constexpr int resolution = 36;
  constexpr int numThreads = 16;
  const auto scenes = makeScenes();
  const std::filesystem::path outputDir = std::filesystem::path("build") / "dynamicrvgdebug";
  std::filesystem::create_directories(outputDir);

#ifdef WRITE_DYNAMIC_RVG_FULL_PATH
  for (const auto &scene : scenes) {
    DynamicRVG<T> dynamicRVG(scene.robot, scene.map, scene.obstacles, resolution, numThreads);
    const bool planned = dynamicRVG.plan(scene.start, scene.goal);
    expect(planned, "plan succeeds for " + scene.name);

    dynamicRVG.drawFullPathAndEndGraph(scene.name);

    const auto scriptPath = outputDir / ("drawFullPathAndEndGraph_" + scene.name + ".py");
    expect(std::filesystem::exists(scriptPath), "drawFullPathAndEndGraph writes script for " + scene.name);
    expect(!dynamicRVG.getExplorationPath().empty(), "plan populates exploration path for " + scene.name);
    expect(dynamicRVG.getGraph().size() > 0, "plan leaves final graph for " + scene.name);
  }
#else
  Utils::print("Full-path generation disabled. Uncomment WRITE_DYNAMIC_RVG_FULL_PATH in mainDebugFullPath.cpp to enable it.");
#endif

  return 0;
}
