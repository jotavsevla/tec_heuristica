#ifndef PHYSARUM_SOLVER_H
#define PHYSARUM_SOLVER_H

#include <vector>
#include <unordered_map>
#include "Types.h"
#include "Route.h"
using namespace std;

class PhysarumSolver {
private:
    // Constantes
    static constexpr int MAX_ITERATIONS = 100;
    static constexpr double MU = 1.3;
    static constexpr double DELTA_T = 0.01;
    static constexpr double EPSILON = 1e-6;
    static constexpr double MIN_FLOW = 0.01;

    // Atributos da classe
    const int numNodes;
    const int numVehicles;
    const double vehicleCapacity;
    const int depot;

    vector<vector<double>> adjacencyMatrix;
    vector<vector<double>> conductivity;
    vector<vector<vector<double>>> vehicleConductivity;
    unordered_map<int, double> demands;

    // Cache de pressões
    vector<vector<vector<double>>> pressureCache;
    vector<vector<bool>> cacheValid;

public:
    // Construtor
    PhysarumSolver(int nodes, int vehicles, double capacity, int depotNode);

    // Métodos públicos
    void addEdge(int from, int to, double weight);
    void setDemand(int node, double demand);
    vector<Route> findRoutes();
    void printSolution(const vector<Route>& routes);
    static double calculateEuclideanDistance(const Point& p1, const Point& p2);

private:
    // Métodos privados
    vector<double> calculatePressures(int source, int target, int vehicleIdx);
    void updateConductivity(const vector<vector<double>>& flux, int vehicleIdx);
    bool isRouteFeasible(const vector<int>& route) const;
};

#endif // PHYSARUM_SOLVER_H