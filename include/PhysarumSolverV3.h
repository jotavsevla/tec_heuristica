#ifndef PHYSARUM_SOLVER_V3_H
#define PHYSARUM_SOLVER_V3_H

#include "PhysarumSolver.h"
#include <vector>
#include <queue>
#include <unordered_set>

// Estrutura para armazenar um movimento de vizinhança
struct NeighborhoodMove {
    int routeIndex1;
    int routeIndex2;
    int nodeIndex1;
    int nodeIndex2;
    double costDelta;  // Mudança no custo total

    NeighborhoodMove(int r1, int r2, int n1, int n2, double delta)
            : routeIndex1(r1), routeIndex2(r2), nodeIndex1(n1), nodeIndex2(n2), costDelta(delta) {}

    // Operador para fila de prioridade (melhor melhora)
    bool operator<(const NeighborhoodMove& other) const {
        return costDelta > other.costDelta;  // Menor custo = maior prioridade
    }
};

// Estrutura para elemento tabu
struct TabuElement {
    int node;
    int routeIndex;
    int iteration;

    TabuElement(int n, int r, int i) : node(n), routeIndex(r), iteration(i) {}
};

class PhysarumSolverV3 : public PhysarumSolver {
private:
    // Parâmetros da busca local e tabu
    int maxIterationsWithoutImprovement;
    int tabuTenure;
    vector<TabuElement> tabuList;

    // Métodos auxiliares para vizinhança
    bool isMoveFeasible(const Route& route1, const Route& route2,
                        int nodeIdx1, int nodeIdx2) const;
    double calculateMoveCost(const Route& route1, const Route& route2,
                             int nodeIdx1, int nodeIdx2) const;
    void applyMove(Route& route1, Route& route2,
                   int nodeIdx1, int nodeIdx2);

    // Métodos de busca local
    NeighborhoodMove findBestInterRouteSwap(const vector<Route>& routes);
    NeighborhoodMove findBestNodeReallocation(const vector<Route>& routes);
    NeighborhoodMove findFirstImprovement(const vector<Route>& routes, bool useSwap);

    // Métodos para busca tabu
    bool isTabu(const NeighborhoodMove& move, double currentCost, double bestCost);
    void updateTabuList(const NeighborhoodMove& move, int currentIteration);
    void adaptTabuTenure(bool improved);
    double calculateTotalDistance(const vector<Route>& routes) const;
    bool validateSolution(const std::vector<Route>& solution);
    double calculateTotalCost(const std::vector<Route>& solution);
    bool hasValidDemands(const std::vector<Route>& solution);
    bool allNodesVisited(const std::vector<Route>& solution);

public:
    PhysarumSolverV3(int nodes, int vehicles, double capacity, int depotNode);

    // Sobrescreve o método findRoutes para incluir busca local e tabu
    vector<Route> findRoutes() override;

    // Métodos de busca local
    vector<Route> applyLocalSearch(vector<Route>& initialSolution,
                                        bool useBestImprovement,
                                        bool useSwap);

    // Método de busca tabu
    vector<Route> applyTabuSearch(vector<Route>& initialSolution);
};

#endif // PHYSARUM_SOLVER_V3_H
