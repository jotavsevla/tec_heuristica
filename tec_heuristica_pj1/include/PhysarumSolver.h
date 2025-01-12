#ifndef PHYSARUM_SOLVER_H
#define PHYSARUM_SOLVER_H

#include <vector>
#include <unordered_map>
#include <utility> // para pair
#include "Types.h"
#include "Route.h"
using namespace std;

class PhysarumSolver {
protected:
    // Constantes
//    static constexpr int MAX_ITERATIONS = 10000;
//    static constexpr double MU = 3.0;
//    static constexpr double DELTA_T = 0.001;
//    static constexpr double EPSILON = 1e-10;
//    static constexpr double MIN_FLOW = 0.005;
//    static constexpr int NETWORK_EVOLUTION_ITERATIONS = 2500;
//    static constexpr double GLOBAL_PRESSURE_WEIGHT = 0.8;

// Parâmetros para busca extensiva
    static constexpr int MAX_ITERATIONS = 75000;         // Mais iterações
    static constexpr double MU = 1.5;                    // Menor reforço inicial
    static constexpr double DELTA_T = 0.00005;          // Mudanças ainda mais graduais
    static constexpr double EPSILON = 1e-6;            // Convergência mais rigorosa
    static constexpr double MIN_FLOW = 0.0005;          // Mantém mais caminhos viáveis
    static constexpr int NETWORK_EVOLUTION_ITERATIONS = 15000;
    static constexpr double GLOBAL_PRESSURE_WEIGHT = 0.95;
    // Atributos da classe
    const int numNodes;
    const int numVehicles;
    const double vehicleCapacity;
    const int depot;

    vector<vector<double>> adjacencyMatrix;
    vector<vector<double>> conductivity;
    vector<vector<vector<double>>> vehicleConductivity;
    unordered_map<int, double> demands;

    // Cache
    vector<vector<vector<double>>> pressureCache;
    vector<vector<bool>> cacheValid;
    vector<vector<double>> globalPressures;
    vector<vector<double>> globalConductivityScores;
    vector<vector<double>> globalPressureCache;
    vector<vector<bool>> globalCacheValid;

    // Métodos privados
    vector<double> calculatePressures(int source, int target, int vehicleIdx);
    double calculateNodeScore(int currentNode, int candidateNode, int vehicleIdx);
    void updateConductivity(const vector<vector<double>>& flux, int vehicleIdx);
    bool isRouteFeasible(const vector<int>& route) const;

    // Métodos para evolução da rede
    void evolveNetwork();
    vector<vector<double>> calculateGlobalPressures();
    double evaluateConnectionQuality(int from, int to, double demand);
    void updateGlobalConductivities(const vector<vector<double>>& pressures);
    bool hasNetworkConverged(const vector<vector<double>>& oldConductivity);
    double calculateGlobalFlow(int from, int to, const vector<vector<double>>& pressures);
    vector<pair<int, int>> findMainPaths();
    Route constructRouteFromPath(const pair<int, int>& path);

public:
    PhysarumSolver(int nodes, int vehicles, double capacity, int depotNode);
    virtual ~PhysarumSolver() = default;
    void addEdge(int from, int to, double weight);
    void setDemand(int node, double demand);
    virtual vector<Route> findRoutes();
    void printSolution(const vector<Route>& routes);
    static double calculateEuclideanDistance(const Point& p1, const Point& p2);
    virtual double calculateRouteSegmentDistance(int from, int to) const;
};

#endif // PHYSARUM_SOLVER_H