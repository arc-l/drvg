#include <DynamicVisibilityGraph/DynamicRVG.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

// Uncomment to generate an overlay script in build/dynamicrvgdebug.
#define WRITE_DYNAMIC_RVG_SCENE_COMPARISON

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

Scene makeDifferentVisibilityScene() {
  const Polygon<T> robot = makeRobot();
  const Polygon<T> map = makeMap(20.);
  Vertex<T> start = robot.getCentroid();
  start.setBounds(0, 0);
  start.setTheta(0);

  return {
      "center_vs_vertices",
      robot,
      map,
      {
          // This thin wall blocks the robot center, while the top and bottom
          // vertices can still see around it and reveal more free space.
          Polygon<T>({
              Vertex<T>(0.25, 0.70),
              Vertex<T>(6.50, 0.70),
              Vertex<T>(6.50, 1.30),
              Vertex<T>(0.25, 1.30)
          }, true)
      },
      start
  };
}

void expect(bool ok, const std::string &name) {
  Utils::print(ok ? "[PASS]" : "[FAIL]", name);
  if (!ok) throw std::runtime_error(name);
}

bool polygonsMatch(const Polygon<T> &lhs, const Polygon<T> &rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  if (!lhs.size()) {
    return true;
  }

  size_t offset = lhs.size();
  for (size_t i = 0; i < static_cast<size_t>(lhs.size()); ++i) {
    if (lhs[static_cast<int>(i)].dist(rhs[0]) < 1e-6) {
      offset = i;
      break;
    }
  }
  if (offset == static_cast<size_t>(lhs.size())) {
    return false;
  }

  for (size_t i = 0; i < static_cast<size_t>(lhs.size()); ++i) {
    if (lhs[static_cast<int>((i + offset) % static_cast<size_t>(lhs.size()))].dist(rhs[static_cast<int>(i)]) > 1e-6) {
      return false;
    }
  }
  return true;
}

void appendPolygonScript(
    std::string &pythonScript,
    int &polygonId,
    const Polygon<T> &polygon,
    const std::string &edgeColor,
    const std::string &fillColor,
    double alpha,
    double lineWidth
) {
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
  pythonScript += "ax.fill(" + prefix + "_x, " + prefix + "_y, color='" + fillColor +
                  "', alpha=" + std::to_string(alpha) + ")\n";
  pythonScript += "ax.plot(" + prefix + "_x, " + prefix + "_y, color='" + edgeColor +
                  "', linewidth=" + std::to_string(lineWidth) + ")\n";
}

void writeComparisonScript(
    const Scene &scene,
    const Polygon<T> &singleVisibleArea,
    const Polygon<T> &allVertexVisibleArea,
    const std::filesystem::path &outputDir
) {
  const std::string suffix = "_" + scene.name;
  const std::string imagePath = (outputDir / ("visibleAreaComparison" + suffix + ".png")).string();
  const std::string scriptPath = (outputDir / ("drawVisibleAreaComparison" + suffix + ".py")).string();
  const Polygon<T> robotFootprint = scene.robot.moveToCopy(
      scene.start.getX(),
      scene.start.getY(),
      scene.start.getTheta()
  );

  std::string pythonScript;
  pythonScript += "import matplotlib.pyplot as plt\n";
  pythonScript += "from matplotlib.patches import Patch\n";
  pythonScript += "fig, ax = plt.subplots(dpi=500)\n";
  pythonScript += "ax.set_facecolor('gainsboro')\n";

  int polygonId = 0;
  appendPolygonScript(pythonScript, polygonId, scene.map, "black", "white", 0.02, 1.0);
  for (const auto &obstacle : scene.obstacles) {
    appendPolygonScript(pythonScript, polygonId, obstacle, "darkslategray", "lightsteelblue", 0.85, 1.2);
  }
  appendPolygonScript(pythonScript, polygonId, singleVisibleArea, "royalblue", "deepskyblue", 0.20, 1.8);
  appendPolygonScript(pythonScript, polygonId, allVertexVisibleArea, "orangered", "gold", 0.24, 1.8);
  appendPolygonScript(pythonScript, polygonId, robotFootprint, "crimson", "mistyrose", 0.35, 1.2);

  pythonScript += "ax.plot([" + std::to_string(scene.start.getX()) + "], [" + std::to_string(scene.start.getY()) +
                  "], 'o', color='crimson', markersize=5)\n";
  pythonScript += "ax.text(" + std::to_string(scene.start.getX()) + ", " + std::to_string(scene.start.getY()) +
                  ", ' start', color='crimson', fontsize=8)\n";
  pythonScript += "legend_handles = [\n";
  pythonScript += "    Patch(facecolor='deepskyblue', edgecolor='royalblue', alpha=0.20, label='scanVisibleArea'),\n";
  pythonScript += "    Patch(facecolor='gold', edgecolor='orangered', alpha=0.24, label='scanFromAllVertices'),\n";
  pythonScript += "    Patch(facecolor='lightsteelblue', edgecolor='darkslategray', alpha=0.85, label='obstacle')\n";
  pythonScript += "]\n";
  pythonScript += "ax.legend(handles=legend_handles, loc='upper left')\n";
  pythonScript += "ax.set_aspect('equal', adjustable='box')\n";
  pythonScript += "plt.title('Visible area comparison')\n";
  pythonScript += "plt.savefig('" + imagePath + "', dpi=500, bbox_inches='tight')\n";

  std::ofstream file(scriptPath);
  if (!file) {
    throw std::runtime_error("Failed to open " + scriptPath);
  }
  file << pythonScript;
}

int main() {
  constexpr int resolution = 36;
  constexpr int numThreads = 16;
  const Scene scene = makeDifferentVisibilityScene();
  const std::filesystem::path outputDir = std::filesystem::path("build") / "dynamicrvgdebug";
  std::filesystem::create_directories(outputDir);

#ifdef WRITE_DYNAMIC_RVG_SCENE_COMPARISON
  DynamicRVG<T> dynamicRVG(scene.robot, scene.map, scene.obstacles, resolution, numThreads);
  dynamicRVG.moverobotdebug(scene.start);

  const Polygon<T> singleVisibleArea = dynamicRVG.scanVisibleArea(scene.start);
  const Polygon<T> allVertexVisibleArea = dynamicRVG.scanFromAllVertices(scene.start);

  expect(singleVisibleArea.size() >= 3, "scanVisibleArea returns a polygon");
  expect(allVertexVisibleArea.size() >= 3, "scanFromAllVertices returns a polygon");
  expect(!polygonsMatch(singleVisibleArea, allVertexVisibleArea),
         "comparison scene produces different visible areas");

  writeComparisonScript(scene, singleVisibleArea, allVertexVisibleArea, outputDir);

  const auto scriptPath = outputDir / ("drawVisibleAreaComparison_" + scene.name + ".py");
  expect(std::filesystem::exists(scriptPath), "comparison script is written");
#else
  Utils::print("Scene comparison generation disabled. Uncomment WRITE_DYNAMIC_RVG_SCENE_COMPARISON in mainDebugScanVisibleArea.cpp to enable it.");
#endif

  return 0;
}
