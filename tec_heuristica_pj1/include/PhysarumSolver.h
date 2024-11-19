#ifndef PHYSARUM_SOLVER_H
#define PHYSARUM_SOLVER_H

#include "Types.h"
#include "Route.h"
#include "Edge.h"
#include <vector>
#include <map>

using namespace std;

class PhysarumSolver {
private:
    int numNodes;
    int numVehicles;
    double vehicleCapacity;
    int depot;
    vector<vector<double>> adjacencyMatrix;
    vector<vector<double>> conductivity;
    vector<vector<vector<double>>> vehicleConductivity;
    map<int, double> demands;

    vector<double> calculatePressures(int source, int target, int vehicleIdx);
    void updateConductivity(const vector<vector<double>>& flux, int vehicleIdx);
    bool isRouteFeasible(const vector<int>& route) const;
    double calculateEuclideanDistance(const Point& p1, const Point& p2);

public:
    PhysarumSolver(int nodes, int vehicles, double capacity, int depotNode = 0);
    void addEdge(int from, int to, double weight);
    void setDemand(int node, double demand);
    std::vector<Route> findRoutes();
    void printSolution(const vector<Route>& routes);
};

#endif // PHYSARUM_SOLVER_H