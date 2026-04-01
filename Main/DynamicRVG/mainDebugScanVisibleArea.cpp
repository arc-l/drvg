#include <DynamicVisibilityGraph/DynamicRVG.h>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

// Uncomment to generate scene-specific visibility scripts in build/dynamicrvgdebug.
#define WRITE_DYNAMIC_RVG_SCENES

using namespace RotationalVisibilityGraph;

typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

struct Scene {
  std::string name;
  Polygon<T> robot, map;
  std::vector<Polygon<T>> obstacles;
  Vertex<T> start;
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

  Scene narrowPassage{
      "narrow_passage",
      robot,
      map,
      {
          Polygon<T>({Vertex<T>(2, -1), Vertex<T>(4, -1), Vertex<T>(4, -2), Vertex<T>(2, -2)}, true),
          Polygon<T>({Vertex<T>(2,  1), Vertex<T>(4,  1), Vertex<T>(4,  2), Vertex<T>(2,  2)}, true)
      },
      start
  };

  Scene centerBlock{
      "center_block",
      robot,
      map,
      {
          Polygon<T>({Vertex<T>(1, -2), Vertex<T>(4, -2), Vertex<T>(4, 2), Vertex<T>(1, 2)}, true)
      },
      start
  };

  Scene zigzag{
      "zigzag",
      robot,
      map,
      {
          Polygon<T>({Vertex<T>(0, -4), Vertex<T>(2, -4), Vertex<T>(2, -1), Vertex<T>(0, -1)}, true),
          Polygon<T>({Vertex<T>(3,  1), Vertex<T>(5,  1), Vertex<T>(5,  4), Vertex<T>(3,  4)}, true),
          Polygon<T>({Vertex<T>(6, -4), Vertex<T>(8, -4), Vertex<T>(8,  0), Vertex<T>(6,  0)}, true)
      },
      start
  };

  return {narrowPassage, centerBlock, zigzag};
}

void expect(bool ok, const std::string &name) {
  Utils::print(ok ? "[PASS]" : "[FAIL]", name);
  if (!ok) throw std::runtime_error(name);
}

int main() {
  constexpr int resolution = 36, numThreads = 16;
  const auto scenes = makeScenes();
  const std::filesystem::path outputDir = std::filesystem::path("build") / "dynamicrvgdebug";
  std::filesystem::create_directories(outputDir);

#ifdef WRITE_DYNAMIC_RVG_SCENES
  for (const auto &scene : scenes) {
    DynamicRVG<T> dynamicRVG(scene.robot, scene.map, scene.obstacles, resolution, numThreads);
    dynamicRVG.plan(
        std::make_shared<Vertex<T>>(scene.start),
        std::make_shared<Vertex<T>>(scene.start)
    );
    dynamicRVG.drawVisibleArea(scene.name);

    const auto scriptPath = outputDir / ("drawVisibleArea_" + scene.name + ".py");
    expect(std::filesystem::exists(scriptPath), "drawVisibleArea writes script for " + scene.name);
  }
#else
  Utils::print("Scene script generation disabled. Uncomment WRITE_DYNAMIC_RVG_SCENES in mainDebugScanVisibleArea.cpp to enable it.");
#endif

  return 0;
}
