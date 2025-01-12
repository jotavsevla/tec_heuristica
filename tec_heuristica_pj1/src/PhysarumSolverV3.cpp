// PhysarumSolverV3.cpp
#include "PhysarumSolverV3.h"
#include <set>
#include <iostream>
#include <algorithm>
#include <limits>

using namespace std;

PhysarumSolverV3::PhysarumSolverV3(int nodes, int vehicles, double capacity, int depotNode)
        : PhysarumSolver(nodes, vehicles, capacity, depotNode) {
    maxIterationsWithoutImprovement = 100;
    tabuTenure = 20;
}


// Implementação dos métodos de vizinhança para PhysarumSolverV3

bool PhysarumSolverV3::isMoveFeasible(const Route& route1, const Route& route2,
                                      int nodeIdx1, int nodeIdx2) const {
    // Verifica se os índices são válidos e não são depósitos
    if (nodeIdx1 <= 0 || static_cast<size_t>(nodeIdx1) >= route1.nodes.size() - 1 ||
        nodeIdx2 <= 0 || static_cast<size_t>(nodeIdx2) >= route2.nodes.size() - 1) {
        return false;
    }

    // Para troca entre rotas, verifica capacidade
    int node1 = route1.nodes[nodeIdx1];
    int node2 = route2.nodes[nodeIdx2];

    double demand1 = demands.at(node1);
    double demand2 = demands.at(node2);

    // Calcula novas demandas após a troca
    double newDemand1 = route1.totalDemand - demand1 + demand2;
    double newDemand2 = route2.totalDemand - demand2 + demand1;

    return newDemand1 <= vehicleCapacity && newDemand2 <= vehicleCapacity;
}

double PhysarumSolverV3::calculateMoveCost(const Route& route1, const Route& route2,
                                           int nodeIdx1, int nodeIdx2) const {
    double currentCost = 0.0;
    double newCost = 0.0;

    // Calcula custo atual
    if (&route1 == &route2) {  // Movimento intra-rota
        // Conexões atuais
        currentCost += calculateRouteSegmentDistance(route1.nodes[nodeIdx1-1], route1.nodes[nodeIdx1]);
        currentCost += calculateRouteSegmentDistance(route1.nodes[nodeIdx1], route1.nodes[nodeIdx1+1]);
        currentCost += calculateRouteSegmentDistance(route1.nodes[nodeIdx2-1], route1.nodes[nodeIdx2]);
        currentCost += calculateRouteSegmentDistance(route1.nodes[nodeIdx2], route1.nodes[nodeIdx2+1]);

        // Novas conexões após troca
        newCost += calculateRouteSegmentDistance(route1.nodes[nodeIdx1-1], route1.nodes[nodeIdx2]);
        newCost += calculateRouteSegmentDistance(route1.nodes[nodeIdx2], route1.nodes[nodeIdx1+1]);
        newCost += calculateRouteSegmentDistance(route1.nodes[nodeIdx2-1], route1.nodes[nodeIdx1]);
        newCost += calculateRouteSegmentDistance(route1.nodes[nodeIdx1], route1.nodes[nodeIdx2+1]);
    } else {  // Movimento inter-rota
        // Custo atual na rota 1
        currentCost += calculateRouteSegmentDistance(route1.nodes[nodeIdx1-1], route1.nodes[nodeIdx1]);
        currentCost += calculateRouteSegmentDistance(route1.nodes[nodeIdx1], route1.nodes[nodeIdx1+1]);

        // Custo atual na rota 2
        currentCost += calculateRouteSegmentDistance(route2.nodes[nodeIdx2-1], route2.nodes[nodeIdx2]);
        currentCost += calculateRouteSegmentDistance(route2.nodes[nodeIdx2], route2.nodes[nodeIdx2+1]);

        // Novo custo após troca
        newCost += calculateRouteSegmentDistance(route1.nodes[nodeIdx1-1], route2.nodes[nodeIdx2]);
        newCost += calculateRouteSegmentDistance(route2.nodes[nodeIdx2], route1.nodes[nodeIdx1+1]);
        newCost += calculateRouteSegmentDistance(route2.nodes[nodeIdx2-1], route1.nodes[nodeIdx1]);
        newCost += calculateRouteSegmentDistance(route1.nodes[nodeIdx1], route2.nodes[nodeIdx2+1]);
    }

    return newCost - currentCost;
}

void PhysarumSolverV3::applyMove(Route& route1, Route& route2,
                                 int nodeIdx1, int nodeIdx2) {
    // Preserva os depósitos no início e fim
    int depot = route1.nodes[0];  // Assume mesmo depósito para ambas rotas

    // Troca os nós
    int node1 = route1.nodes[nodeIdx1];
    int node2 = route2.nodes[nodeIdx2];

    double demand1 = demands.at(node1);
    double demand2 = demands.at(node2);

    // Atualiza demandas e nós
    route1.totalDemand = route1.totalDemand - demand1 + demand2;
    route2.totalDemand = route2.totalDemand - demand2 + demand1;

    // Garante início/fim no depósito
    route1.nodes[0] = depot;
    route1.nodes[route1.nodes.size()-1] = depot;
    route2.nodes[0] = depot;
    route2.nodes[route2.nodes.size()-1] = depot;

    route1.nodes[nodeIdx1] = node2;
    route2.nodes[nodeIdx2] = node1;

    // Recalcula distâncias
    route1.totalDistance = 0;
    route2.totalDistance = 0;

    for (size_t i = 0; i < route1.nodes.size() - 1; i++) {
        route1.totalDistance += calculateRouteSegmentDistance(route1.nodes[i], route1.nodes[i+1]);
    }

    for (size_t i = 0; i < route2.nodes.size() - 1; i++) {
        route2.totalDistance += calculateRouteSegmentDistance(route2.nodes[i], route2.nodes[i+1]);
    }
}

NeighborhoodMove PhysarumSolverV3::findBestInterRouteSwap(const vector<Route>& routes) {
    NeighborhoodMove bestMove(-1, -1, -1, -1, numeric_limits<double>::max());

    try {
        for (size_t i = 0; i < routes.size(); i++) {
            for (size_t j = i + 1; j < routes.size(); j++) {
                if (routes[i].nodes.empty() || routes[j].nodes.empty()) {
                    continue;
                }

                const Route& route1 = routes[i];
                const Route& route2 = routes[j];

                for (size_t nodeIdx1 = 1; nodeIdx1 < route1.nodes.size() - 1; nodeIdx1++) {
                    for (size_t nodeIdx2 = 1; nodeIdx2 < route2.nodes.size() - 1; nodeIdx2++) {
                        if (isMoveFeasible(route1, route2, nodeIdx1, nodeIdx2)) {
                            double deltaCost = calculateMoveCost(route1, route2, nodeIdx1, nodeIdx2);
                            if (deltaCost < bestMove.costDelta) {
                                bestMove = NeighborhoodMove(i, j, nodeIdx1, nodeIdx2, deltaCost);
                            }
                        }
                    }
                }
            }
        }
    } catch (const exception& e) {
        cerr << "Erro em findBestInterRouteSwap: " << e.what() << endl;
        throw;
    }

    return bestMove;
}

NeighborhoodMove PhysarumSolverV3::findBestNodeReallocation(const std::vector<Route>& routes) {
    NeighborhoodMove bestMove(-1, -1, -1, -1, std::numeric_limits<double>::max());

    for (size_t i = 0; i < routes.size(); i++) {
        const Route& fromRoute = routes[i];

        // Para cada nó na rota atual (exceto depósito)
        for (size_t nodeIdx = 1; nodeIdx < fromRoute.nodes.size() - 1; nodeIdx++) {
            int node = fromRoute.nodes[nodeIdx];
            double nodeDemand = demands.at(node);

            // Tenta inserir em todas as outras rotas
            for (size_t j = 0; j < routes.size(); j++) {
                if (i == j) continue;
                const Route& toRoute = routes[j];

                // Verifica se a rota de destino pode receber o nó
                if (toRoute.totalDemand + nodeDemand > vehicleCapacity) {
                    continue;
                }

                // Tenta inserir em todas as posições possíveis
                for (size_t insertPos = 1; insertPos < toRoute.nodes.size(); insertPos++) {
                    // Calcula o custo da realocação
                    double removalCost = calculateRouteSegmentDistance(fromRoute.nodes[nodeIdx-1], fromRoute.nodes[nodeIdx]) +
                                         calculateRouteSegmentDistance(fromRoute.nodes[nodeIdx], fromRoute.nodes[nodeIdx+1]) -
                                         calculateRouteSegmentDistance(fromRoute.nodes[nodeIdx-1], fromRoute.nodes[nodeIdx+1]);

                    double insertionCost = calculateRouteSegmentDistance(toRoute.nodes[insertPos-1], node) +
                                           calculateRouteSegmentDistance(node, toRoute.nodes[insertPos]) -
                                           calculateRouteSegmentDistance(toRoute.nodes[insertPos-1], toRoute.nodes[insertPos]);

                    double totalDeltaCost = insertionCost - removalCost;

                    // Atualiza melhor movimento se encontrou um custo menor
                    if (totalDeltaCost < bestMove.costDelta) {
                        bestMove = NeighborhoodMove(i, j, nodeIdx, insertPos, totalDeltaCost);
                    }
                }
            }
        }
    }

    return bestMove;
}
// Implementação da busca local
vector<Route> PhysarumSolverV3::applyLocalSearch(vector<Route>& initialSolution,
                                                 bool useBestImprovement,
                                                 bool useSwap) {
    if (initialSolution.empty())
        throw runtime_error("solução inicial vazia em applyLocalSearch");

    vector<Route> currentSolution = initialSolution;

    for (size_t i = 0; i < currentSolution.size(); i++) {

        const auto& route = currentSolution[i];

        if (route.nodes.empty()) {
            throw runtime_error("Rota " + to_string(i) + " vazia");
        }
        if (route.nodes.front() != depot || route.nodes.back() != depot) {
            throw runtime_error("Rota " + to_string(i) + " não começa/termina no depósito");
        }

        // Verifica demanda e distância
        double totalDemand = 0;
        for (size_t j = 1; j < route.nodes.size() - 1; j++) {
            if (route.nodes[j] >= numNodes) {
                throw runtime_error("Nó inválido na rota " + to_string(i));
            }
            totalDemand += demands.at(route.nodes[j]);
        }
        if (totalDemand > vehicleCapacity) {
            throw runtime_error("Capacidade excedida na rota " + to_string(i));
        }
    }


    bool improved;
    int iterations = 0;

    try {
        do {
            improved = false;
            if (useBestImprovement) {
                NeighborhoodMove bestMove = useSwap ?
                                            findBestInterRouteSwap(currentSolution) :
                                            findBestNodeReallocation(currentSolution);

                if (bestMove.routeIndex1 != -1 && bestMove.costDelta < 0 &&
                    static_cast<size_t>(bestMove.routeIndex1) < currentSolution.size() &&
                    static_cast<size_t>(bestMove.routeIndex2) < currentSolution.size()) {

                    applyMove(currentSolution[bestMove.routeIndex1],
                              currentSolution[bestMove.routeIndex2],
                              bestMove.nodeIndex1,
                              bestMove.nodeIndex2);
                    improved = true;
                }
            } else {
                NeighborhoodMove move = findFirstImprovement(currentSolution, useSwap);
                if (move.routeIndex1 != -1) {
                    if (static_cast<size_t>(move.routeIndex1) >= currentSolution.size() ||
                        static_cast<size_t>(move.routeIndex2) >= currentSolution.size()) {
                        throw runtime_error("Índice de rota inválido");
                    }
                    applyMove(currentSolution[move.routeIndex1],
                              currentSolution[move.routeIndex2],
                              move.nodeIndex1,
                              move.nodeIndex2);
                    improved = true;
                }
            }
            iterations++;
        } while (improved && iterations < maxIterationsWithoutImprovement);
    } catch (const exception& e) {
        cerr << "Erro durante busca local: " << e.what() << endl;
        throw;
    }

    return currentSolution;
}

// Implementação da busca da primeira melhora
NeighborhoodMove PhysarumSolverV3::findFirstImprovement(const vector<Route>& routes, bool useSwap) {

    try {
        if (useSwap) {
            for (size_t i = 0; i < routes.size(); i++) {
                for (size_t j = i + 1; j < routes.size(); j++) {
                    if (routes[i].nodes.empty() || routes[j].nodes.empty()) continue;

                    for (size_t pos1 = 1; pos1 < routes[i].nodes.size() - 1; pos1++) {
                        for (size_t pos2 = 1; pos2 < routes[j].nodes.size() - 1; pos2++) {
                            if (isMoveFeasible(routes[i], routes[j], pos1, pos2)) {
                                double deltaCost = calculateMoveCost(routes[i], routes[j], pos1, pos2);
                                if (deltaCost < 0) {
                                    return NeighborhoodMove(i, j, pos1, pos2, deltaCost);
                                }
                            }
                        }
                    }
                }
            }
        } else {
            for (size_t i = 0; i < routes.size(); i++) {
                if (routes[i].nodes.empty()) continue;

                for (size_t pos1 = 1; pos1 < routes[i].nodes.size() - 1; pos1++) {
                    for (size_t j = 0; j < routes.size(); j++) {
                        if (i == j || routes[j].nodes.empty()) continue;

                        for (size_t pos2 = 1; pos2 < routes[j].nodes.size(); pos2++) {
                            int nodeToMove = routes[i].nodes[pos1];
                            if (nodeToMove < 0 || nodeToMove >= numNodes) {
                                continue;
                            }

                            double nodeDemand = demands.at(nodeToMove);
                            if (routes[j].totalDemand + nodeDemand <= vehicleCapacity) {
                                double deltaCost = calculateMoveCost(routes[i], routes[j], pos1, pos2);
                                if (deltaCost < 0) {
                                    return NeighborhoodMove(i, j, pos1, pos2, deltaCost);
                                }
                            }
                        }
                    }
                }
            }
        }
    } catch (const exception& e) {
        throw;
    }

    return NeighborhoodMove(-1, -1, -1, -1, 0);
}

// Implementação da busca tabu
vector<Route> PhysarumSolverV3::applyTabuSearch(vector<Route>& initialSolution) {
    vector<Route> currentSolution = initialSolution;
    vector<Route> bestSolution = initialSolution;
    double bestCost = 0;

    // Calcula custo inicial
    for (const auto& route : bestSolution) {
        bestCost += route.totalDistance;
    }

    int iterationsWithoutImprovement = 0;
    int iteration = 0;

    while (iterationsWithoutImprovement < maxIterationsWithoutImprovement) {
        NeighborhoodMove bestMove(-1, -1, -1, -1, numeric_limits<double>::max());

        // Encontra o melhor movimento não-tabu
        for (size_t i = 0; i < currentSolution.size(); i++) {
            for (size_t j = 0; j < currentSolution.size(); j++) {
                if (i != j) {
                    for (size_t pos1 = 1; pos1 < currentSolution[i].nodes.size() - 1; pos1++) {
                        for (size_t pos2 = 1; pos2 < currentSolution[j].nodes.size() - 1; pos2++) {
                            if (isMoveFeasible(currentSolution[i], currentSolution[j], pos1, pos2)) {
                                double deltaCost = calculateMoveCost(currentSolution[i], currentSolution[j], pos1, pos2);

                                // Verifica se movimento é tabu
                                bool isTabuMove = isTabu(NeighborhoodMove(i, j, pos1, pos2, deltaCost),
                                                         bestCost + deltaCost, bestCost);

                                if (!isTabuMove && deltaCost < bestMove.costDelta) {
                                    bestMove = NeighborhoodMove(i, j, pos1, pos2, deltaCost);
                                }
                            }
                        }
                    }
                }
            }
        }

        if (bestMove.routeIndex1 != -1) {
            // Aplica o melhor movimento encontrado
            applyMove(currentSolution[bestMove.routeIndex1],
                      currentSolution[bestMove.routeIndex2],
                      bestMove.nodeIndex1,
                      bestMove.nodeIndex2);

            // Atualiza a lista tabu
            updateTabuList(bestMove, iteration);

            // Calcula novo custo total
            double currentCost = 0;
            for (const auto& route : currentSolution) {
                currentCost += route.totalDistance;
            }

            // Atualiza melhor solução se necessário
            if (currentCost < bestCost) {
                bestSolution = currentSolution;
                bestCost = currentCost;
                iterationsWithoutImprovement = 0;
                adaptTabuTenure(true);
            } else {
                iterationsWithoutImprovement++;
                adaptTabuTenure(false);
            }
        } else {
            iterationsWithoutImprovement++;
        }

        iteration++;
    }

    return bestSolution;
}

// Implementação dos métodos auxiliares da busca tabu
bool PhysarumSolverV3::isTabu(const NeighborhoodMove& move, double currentCost, double bestCost) {
    // Critério de aspiração: aceita movimento tabu se produzir melhor solução já encontrada
    if (currentCost < bestCost) {
        return false;
    }

    // Verifica se o movimento está na lista tabu
    for (const auto& tabuElement : tabuList) {
        if ((tabuElement.node == move.routeIndex1 && tabuElement.routeIndex == move.routeIndex2) ||
            (tabuElement.node == move.routeIndex2 && tabuElement.routeIndex == move.routeIndex1)) {
            return true;
        }
    }

    return false;
}

void PhysarumSolverV3::updateTabuList(const NeighborhoodMove& move, int currentIteration) {
    // Remove elementos antigos
    auto newEnd = remove_if(tabuList.begin(), tabuList.end(),
                            [currentIteration, this](const TabuElement& element) {
                                return currentIteration - element.iteration >= tabuTenure;
                            });
    tabuList.erase(newEnd, tabuList.end());

    // Adiciona novo elemento
    tabuList.emplace_back(move.routeIndex1, move.routeIndex2, currentIteration);
    tabuList.emplace_back(move.routeIndex2, move.routeIndex1, currentIteration);
}

void PhysarumSolverV3::adaptTabuTenure(bool improved) {
    if (improved) {
        // Reduz o tamanho da lista tabu se encontrou melhoria
        tabuTenure = max(10, tabuTenure - 1);
    } else {
        // Aumenta o tamanho da lista tabu se está preso em uma região
        tabuTenure = min(30, tabuTenure + 1);
    }
}

vector<Route> PhysarumSolverV3::findRoutes() {
    cout << "Iniciando findRoutes V3..." << endl;

    vector<Route> initialSolution = PhysarumSolver::findRoutes();

    vector<vector<Route>> allSolutions;

    try {
        auto sol1 = applyLocalSearch(initialSolution, false, true);
        allSolutions.push_back(sol1);

        auto sol2 = applyLocalSearch(initialSolution, false, false);
        allSolutions.push_back(sol2);

        auto sol3 = applyLocalSearch(initialSolution, true, true);
        allSolutions.push_back(sol3);

        auto sol4 = applyLocalSearch(initialSolution, true, false);
        allSolutions.push_back(sol4);

        vector<Route> bestSolution = initialSolution;
        double bestCost = calculateTotalCost(initialSolution);

        for (const auto& solution : allSolutions) {
            if (validateSolution(solution)) {
                double cost = calculateTotalCost(solution);
                if (cost < bestCost) {
                    bestCost = cost;
                    bestSolution = solution;
                }
            }
        }

        return applyTabuSearch(bestSolution);
    } catch (const exception& e) {
        cerr << "Erro em findRoutes V3: " << e.what() << endl;
        throw;
    }
}

// Adicionar estas funções auxiliares:
bool PhysarumSolverV3::validateSolution(const vector<Route>& solution) {
    set<int> visitedNodes;

    for (const Route& route : solution) {
        if (route.nodes.front() != depot || route.nodes.back() != depot) {
            return false;
        }

        double totalDemand = 0;
        for (size_t i = 1; i < route.nodes.size() - 1; i++) {
            visitedNodes.insert(route.nodes[i]);
            totalDemand += demands.at(route.nodes[i]);
        }

        if (totalDemand > vehicleCapacity) {
            return false;
        }
    }

    return visitedNodes.size() == static_cast<size_t>(numNodes - 1);
}

double PhysarumSolverV3::calculateTotalCost(const vector<Route>& solution) {
    double totalCost = 0;
    for (const Route& route : solution) {
        for (size_t i = 0; i < route.nodes.size() - 1; i++) {
            totalCost += calculateRouteSegmentDistance(route.nodes[i], route.nodes[i+1]);
        }
    }
    return totalCost;
}