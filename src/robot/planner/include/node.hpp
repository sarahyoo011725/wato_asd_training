#ifndef aNODE_HPP_
#define aNODE_HPP_

#include <functional>
#include <memory>
#include <cmath>

struct Point {
    double x, y;
    Point(double x_, double y_) : x(x_), y(y_) {};
    Point() : x(0), y(0) {};

    double distance_to(Point other) {
        return std::hypot(x - other.x, y - other.y);
    }

    bool operator==(const Point &other) const
    {
        return (x == other.x && y == other.y);
    }

    struct Hash {
        std::size_t operator()(const Point &idx) const {
            return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
        }
    };
};


class aNode {
    public:
    Point point;
    double f = -1; // f = g + h
    double g = -1; // cost from start to this node 
    double h = -1; // herustic distance to goal
    aNode *parent = nullptr;

    aNode() : point() {}
    aNode(int x, int y) : point(x, y) {}
    aNode(const Point& p) : point(p) {}
    aNode(const Point& p, double f_, double g_, double h_) 
        : point(p), f(f_), g(g_), h(h_) {}

    bool operator==(const aNode& other) const {
        return point == other.point;
    }

    struct CompareNode {
        bool operator()(const aNode *a, const aNode *b) const {
            return a->f > b->f;
        }
    };

    struct Hash {
        std::size_t operator()(const aNode& n) const {
            return std::hash<int>()(n.point.x) ^ (std::hash<int>()(n.point.y) << 1);
        }
    };
};
#endif