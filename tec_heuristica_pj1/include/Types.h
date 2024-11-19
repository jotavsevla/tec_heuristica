#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <map>

using namespace std;

typedef struct Point {
    double x;
    double y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
}Point;

struct XMLProblemData {
    vector<Point> nodes;
    map<int, double> demands;
    int depot;
    double vehicleCapacity;
    int numVehicles;
};

#endif // TYPES_H