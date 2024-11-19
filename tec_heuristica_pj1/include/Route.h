#ifndef ROUTE_H
#define ROUTE_H

#include <vector>
using namespace std;
struct Route {
    vector<int> nodes;
    double totalDemand;
    double totalDistance;
    Route();
};

#endif // ROUTE_H