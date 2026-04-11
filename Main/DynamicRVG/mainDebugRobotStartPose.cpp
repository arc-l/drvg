#include <algorithm>
#include <cmath>
#include <memory>
#include <Utils/Utils.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Utils.h>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace RotationalVisibilityGraph;

using T = double;

namespace {

std::string formatValue(T value) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(3);
  stream << value;
  return stream.str();
}

template <typename U>
U maxSpan(const Polygon<U> &polygon) {
  const auto x = polygon.getX();
  const auto y = polygon.getY();
  U minX = x.front();
  U maxX = x.front();
  U minY = y.front();
  U maxY = y.front();
  for (size_t i = 1; i < x.size(); ++i) {
    minX = std::min(minX, x[i]);
    maxX = std::max(maxX, x[i]);
    minY = std::min(minY, y[i]);
    maxY = std::max(maxY, y[i]);
  }
  const U span = std::max(maxX - minX, maxY - minY);
  return span > 0 ? span : static_cast<U>(1);
}

}  // namespace

int main(int argc, char *argv[]) {
  if (argc < 2) {
    throw std::runtime_error("Usage: mainDebugRobotStartPose <config.xml> [imagePath]");
  }

  tinyxml2::XMLDocument pt;
  if (pt.LoadFile(argv[1]) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to load config file");
  }

  const Polygon<T> robot = RotationalVisibilityGraph::Utils::getRobot<T>(pt);
  const auto plannerSettings = pt.RootElement()->FirstChildElement("plannerSettings");
  if (!plannerSettings) {
    throw std::runtime_error("plannerSettings not found");
  }
  const auto startElement = plannerSettings->FirstChildElement("start");
  if (!startElement || !startElement->FirstChildElement("Vertex")) {
    throw std::runtime_error("plannerSettings.start.Vertex not found");
  }
  const std::shared_ptr<Vertex<T>> start = RotationalVisibilityGraph::Utils::getVertex<T>(
      *startElement->FirstChildElement("Vertex")
  );

  const bool startHasTheta = start->hasTheta();
  const T startTheta = startHasTheta ? start->getTheta() : static_cast<T>(0);
  const Polygon<T> robotAtStart = robot.moveToCopy(start->getX(), start->getY(), startTheta);
  const Vertex<T> &referencePoint = robot.getCentroid();
  const T localArrowLength = static_cast<T>(0.3) * maxSpan(robot);
  const T worldArrowLength = static_cast<T>(0.3) * maxSpan(robotAtStart);

  const std::filesystem::path outputDir = std::filesystem::path("build") / "dynamicrvgdebug";
  std::filesystem::create_directories(outputDir);
  const std::filesystem::path configPath(argv[1]);
  const std::string outputStem = configPath.stem().string();
  const std::string imagePath = argc > 2
      ? std::filesystem::path(argv[2]).string()
      : (outputDir / ("robotStartPose_" + outputStem + ".png")).string();
  const std::string scriptPath = (outputDir / ("drawRobotStartPose_" + outputStem + ".py")).string();
  const std::filesystem::path imageOutputPath(imagePath);
  if (!imageOutputPath.parent_path().empty()) {
    std::filesystem::create_directories(imageOutputPath.parent_path());
  }

  std::string pythonScript;
  pythonScript += "import matplotlib.pyplot as plt\n";
  pythonScript += "fig, (ax_local, ax_world) = plt.subplots(1, 2, figsize=(12, 6), dpi=250)\n";
  pythonScript += "for ax in (ax_local, ax_world):\n";
  pythonScript += "    ax.set_facecolor('gainsboro')\n";
  pythonScript += "    ax.set_aspect('equal', adjustable='box')\n";
  pythonScript += "    ax.grid(color='white', linewidth=0.8)\n";

  int polygonId = 0;
  const auto appendPolygon = [&](const Polygon<T> &polygon,
                                 const std::string &axisName,
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
      if (i + 1 < x.size()) {
        pythonScript += ", ";
      }
    }
    pythonScript += "]\n";
    pythonScript += prefix + "_y = [";
    for (size_t i = 0; i < y.size(); ++i) {
      pythonScript += std::to_string(y[i]);
      if (i + 1 < y.size()) {
        pythonScript += ", ";
      }
    }
    pythonScript += "]\n";
    pythonScript += axisName + ".fill(" + prefix + "_x, " + prefix + "_y, color='" + fillColor +
                    "', alpha=" + std::to_string(alpha) + ")\n";
    pythonScript += axisName + ".plot(" + prefix + "_x, " + prefix + "_y, '-o', color='" + edgeColor +
                    "', markersize=3.0, linewidth=" + std::to_string(lineWidth) + ")\n";
  };

  const auto appendMarker = [&](const std::string &axisName,
                                const Vertex<T> &marker,
                                const std::string &color,
                                const std::string &label) {
    pythonScript += axisName + ".plot([" + std::to_string(marker.getX()) + "], [" +
                    std::to_string(marker.getY()) + "], 'o', color='" + color + "', markersize=7)\n";
    pythonScript += axisName + ".text(" + std::to_string(marker.getX()) + ", " +
                    std::to_string(marker.getY()) + ", '" + label +
                    "', color='" + color + "', fontsize=9)\n";
  };

  const auto appendHeading = [&](const std::string &axisName,
                                 const Vertex<T> &origin,
                                 T theta,
                                 T arrowLength,
                                 const std::string &color) {
    const T dx = std::cos(theta) * arrowLength;
    const T dy = std::sin(theta) * arrowLength;
    pythonScript += axisName + ".arrow(" + std::to_string(origin.getX()) + ", " +
                    std::to_string(origin.getY()) + ", " + std::to_string(dx) + ", " +
                    std::to_string(dy) + ", color='" + color +
                    "', width=" + std::to_string(arrowLength * static_cast<T>(0.03)) +
                    ", head_width=" + std::to_string(arrowLength * static_cast<T>(0.12)) +
                    ", length_includes_head=True)\n";
  };

  appendPolygon(robot, "ax_local", "crimson", "mistyrose", 0.55, 1.4);
  appendMarker("ax_local", referencePoint, "navy", " start/reference");
  appendHeading("ax_local", referencePoint, static_cast<T>(0), localArrowLength, "navy");
  pythonScript += "ax_local.set_title('Robot Coordinates')\n";
  pythonScript += "ax_local.set_xlabel('x')\n";
  pythonScript += "ax_local.set_ylabel('y')\n";

  appendPolygon(robotAtStart, "ax_world", "forestgreen", "honeydew", 0.55, 1.4);
  appendMarker("ax_world", *start, "crimson", " start");
  appendHeading("ax_world", *start, startTheta, worldArrowLength, "crimson");
  pythonScript += "ax_world.set_title('Planner Start Pose')\n";
  pythonScript += "ax_world.set_xlabel('x')\n";
  pythonScript += "ax_world.set_ylabel('y')\n";

  pythonScript += "fig.suptitle('Robot start anchor for " + outputStem + "')\n";
  pythonScript += "fig.text(0.5, 0.02, 'Local reference point: (" + formatValue(referencePoint.getX()) + ", " +
                  formatValue(referencePoint.getY()) + ")    Planner start: (" +
                  formatValue(start->getX()) + ", " + formatValue(start->getY()) + "), theta=" +
                  (startHasTheta ? formatValue(startTheta) : std::string("0.000 (default)")) +
                  "', ha='center', fontsize=9)\n";
  pythonScript += "plt.tight_layout(rect=[0, 0.05, 1, 0.95])\n";
  pythonScript += "plt.savefig('" + imagePath + "', dpi=250, bbox_inches='tight')\n";

  Utils::writeStringToFile<T>(pythonScript, scriptPath);
  Utils::print("Wrote", scriptPath);
  if (!Utils::runPythonScriptAndRemove<T>(scriptPath)) {
    throw std::runtime_error("Failed to render robot start pose");
  }
  Utils::print("Saved", imagePath);

  return 0;
}
