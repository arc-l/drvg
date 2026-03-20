#include <iostream>
#include <DynamicVisibilityGraph/DynamicRVG.h>

using namespace RotationalVisibilityGraph;
using namespace Utils;

typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

int main() {
    println("Testing DynamicRVG...");
    Polygon<T> robot =
        Polygon<T>(
            std::vector<Vertex<T>>{
                Vertex<T>(-3, 0),
                Vertex<T>(-2, 0),
                Vertex<T>(-2, 2),
                Vertex<T>(-3, 2)
            },
            false);
    std::vector<Vertex<T>> vertices = {
        Vertex<T>(2, -1),
        Vertex<T>(4, -1),
        Vertex<T>(4, -2),
        Vertex<T>(2, -2)
    };
    Polygon<T> polygon(vertices, true);
    std::vector<Vertex<T>> vertices1 = {
        Vertex<T>(2, 1),
        Vertex<T>(4, 1),
        Vertex<T>(4, 2),
        Vertex<T>(2, 2)
    };
    Polygon<T> polygon1 = Polygon(vertices1, true);
    T map_size = 20.;
    std::vector<Vertex<T>> vertices2 = {
        Vertex<T>(-map_size / 2., -map_size / 2.),
        Vertex<T>(map_size / 2., -map_size / 2.),
        Vertex<T>(map_size / 2., map_size / 2.),
        Vertex<T>(-map_size / 2., map_size / 2.)
    };
    Polygon<T> map = Polygon<T>(vertices2, true);
    std::vector<Polygon<T>> obstacles = {polygon, polygon1};
    std::vector<int> resolutions = {36}; // let us test using a low resolution
    Vertex<T> start = robot.getCentroid();
    start.setBounds(0, 0);
    start.setTheta(0);
    Vertex<T> goal(8, 0, PI/2., PI/2., PI/2);
    std::shared_ptr<Vertex<T>> startPtr = std::make_shared<Vertex<T>>(start);
    std::shared_ptr<Vertex<T>> goalPtr = std::make_shared<Vertex<T>>(goal);

    // We can test the scanVisibleArea, buildVisibilityGraph, and mergeVisibilityGraph functions separately before testing the plan function
    //test scanVisibleArea
    DynamicRVG<T> dynamicRVG = DynamicRVG<T>(robot, map, obstacles, resolutions[0], 16);
    dynamicRVG.scanVisibleArea();

    for (int resolution : resolutions) {
        DynamicRVG<T> dynamicRVG = DynamicRVG<T>(robot, map, obstacles, resolution, 16);
        dynamicRVG.plan(start, goal);
    }

    return 0;
}