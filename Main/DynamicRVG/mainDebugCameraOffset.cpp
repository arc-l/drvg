#include <DynamicVisibilityGraph/DynamicRVG.h>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#define WRITE_DYNAMIC_RVG_CAMERA_OFFSET

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

Vertex<T> makeCameraOffset() {
  return Vertex<T>(1.5, 0.0);
}

std::vector<Scene> makeScenes() {
  const Polygon<T> robot = makeRobot();
  const Polygon<T> map = makeMap(20.);

  Vertex<T> startFacingForward = robot.getCentroid();
  startFacingForward.setBounds(0, 0);
  startFacingForward.setTheta(0);

  Vertex<T> startFacingUp = robot.getCentroid();
  startFacingUp.setBounds(PI / 2, PI / 2);
  startFacingUp.setTheta(PI / 2);

  return {
      {
          "camera_offset_forward",
          robot,
          map,
          {
              Polygon<T>({Vertex<T>(1, -2), Vertex<T>(4, -2), Vertex<T>(4, 2), Vertex<T>(1, 2)}, true)
          },
          startFacingForward
      },
      {
          "camera_offset_rotated",
          robot,
          map,
          {
              Polygon<T>({Vertex<T>(-1, 2), Vertex<T>(2, 2), Vertex<T>(2, 5), Vertex<T>(-1, 5)}, true)
          },
          startFacingUp
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
  const Vertex<T> cameraOffset = makeCameraOffset();
  const std::filesystem::path outputDir = std::filesystem::path("build") / "dynamicrvgdebug";
  std::filesystem::create_directories(outputDir);

#ifdef WRITE_DYNAMIC_RVG_CAMERA_OFFSET
  for (const auto &scene : scenes) {
    DynamicRVG<T> dynamicRVG(scene.robot, scene.map, scene.obstacles, resolution, numThreads);
    dynamicRVG.moverobotdebug(scene.start);
    dynamicRVG.scanVisibleArea(scene.start);
    dynamicRVG.drawVisibleArea(scene.name + "_no_offset");

    dynamicRVG.setCameraOffset(cameraOffset);
    dynamicRVG.scanVisibleArea(scene.start);
    dynamicRVG.drawVisibleArea(scene.name + "_with_offset");

    const auto noOffsetScriptPath = outputDir / ("drawVisibleArea_" + scene.name + "_no_offset.py");
    const auto withOffsetScriptPath = outputDir / ("drawVisibleArea_" + scene.name + "_with_offset.py");
    expect(std::filesystem::exists(noOffsetScriptPath), "drawVisibleArea writes no-offset script for " + scene.name);
    expect(std::filesystem::exists(withOffsetScriptPath), "drawVisibleArea writes with-offset script for " + scene.name);
  }
#else
  Utils::print("Camera-offset generation disabled. Uncomment WRITE_DYNAMIC_RVG_CAMERA_OFFSET in mainDebugCameraOffset.cpp to enable it.");
#endif

  return 0;
}
