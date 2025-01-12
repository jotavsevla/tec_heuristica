#include "../include/PhysarumSolver.h"
#include <iostream>
#include <set>
#include <cmath>
#include <algorithm>
#include <cstdlib>

using namespace std;

// static constexpr int MAX_ITERATIONS = 5000;    // Mais iterações
// static constexpr double MU = 2.5;              // Reforço mais forte
// static constexpr double DELTA_T = 0.005;       // Mudanças mais graduais
// static constexpr double EPSILON = 1e-8;        // Convergência mais precisa
// static constexpr double MIN_FLOW = 0.008;      // Mantém mais caminhos viáveis


PhysarumSolver::PhysarumSolver(int nodes, int vehicles, double capacity, int depotNode)
        : numNodes(nodes), numVehicles(vehicles), vehicleCapacity(capacity), depot(depotNode) {

    // Inicializações existentes
    adjacencyMatrix.resize(numNodes, vector<double>(numNodes, 0.0));
    conductivity.resize(numNodes, vector<double>(numNodes, 1.0));
    vehicleConductivity.resize(numVehicles,
                               vector<vector<double>>(numNodes, vector<double>(numNodes, 1.0)));

    // Cache para pressões locais
    pressureCache.resize(numVehicles);
    for (auto& cache : pressureCache) {
        cache.resize(numNodes, vector<double>(numNodes, 0.0));
    }
    cacheValid.resize(numVehicles, vector<bool>(numNodes, false));

    // Novas estruturas para evolução da rede
    globalPressures.resize(numNodes, vector<double>(numNodes, 0.0));
    globalConductivityScores.resize(numNodes, vector<double>(numNodes, 0.0));
    globalPressureCache.resize(numNodes, vector<double>(numNodes, 0.0));
    globalCacheValid.resize(numNodes, vector<bool>(numNodes, false));
}

void PhysarumSolver::addEdge(int from, int to, double weight) {
    if (from >= 0 && from < numNodes && to >= 0 && to < numNodes) {
        adjacencyMatrix[from][to] = weight;
        adjacencyMatrix[to][from] = weight;
    }
}

void PhysarumSolver::setDemand(int node, double demand) {
    if (node >= 0 && node < numNodes) {
        demands[node] = demand;
    }
}

vector<double> PhysarumSolver::calculatePressures(int source, int target, int vehicleIdx) {
    vector<double> pressures(numNodes, 0.0);
    vector<double> previousPressures(numNodes);
    pressures[source] = 1.0;
    pressures[target] = 0.0;

    // Adicionando fator de decaimento por distância
    vector<double> distanceWeights(numNodes, 0.0);
    for(int i = 0; i < numNodes; i++) {
        if(adjacencyMatrix[source][i] > 0) {
            distanceWeights[i] = 1.0 / pow(adjacencyMatrix[source][i], 1.5);
        }
    }

    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        previousPressures = pressures;
        bool converged = true;

        for (int i = 0; i < numNodes; ++i) {
            if (i == source || i == target) continue;

            double sumWeightedConductivity = 0.0;
            double sumWeightedPressureFlow = 0.0;

            for (int j = 0; j < numNodes; ++j) {
                if (adjacencyMatrix[i][j] > 0) {
                    double weightedConductivity = vehicleConductivity[vehicleIdx][i][j] *
                                                  distanceWeights[j];
                    sumWeightedConductivity += weightedConductivity;
                    sumWeightedPressureFlow += weightedConductivity * pressures[j];
                }
            }

            if (sumWeightedConductivity > 0) {
                double newPressure = sumWeightedPressureFlow / sumWeightedConductivity;
                if (abs(newPressure - previousPressures[i]) > EPSILON) {
                    converged = false;
                }
                pressures[i] = newPressure;
            }
        }

        if (converged) break;
    }

    return pressures;
}

double PhysarumSolver::calculateNodeScore(int currentNode, int candidateNode, int vehicleIdx) {
    double distance = adjacencyMatrix[currentNode][candidateNode];
    double conductivity = vehicleConductivity[vehicleIdx][currentNode][candidateNode];
    auto pressures = calculatePressures(currentNode, candidateNode, vehicleIdx);
    double flow = conductivity * abs(pressures[currentNode] - pressures[candidateNode]);

    return (conductivity * MU + flow * 4) / pow(distance, 1.8);
}

void PhysarumSolver::updateConductivity(const vector<vector<double>>& flux, int vehicleIdx) {
    for (int i = 0; i < numNodes; ++i) {
        for (int j = 0; j < numNodes; ++j) {
            if (adjacencyMatrix[i][j] > 0) {
                double current = vehicleConductivity[vehicleIdx][i][j];
                double flow = flux[i][j];
                double newConductivity = (1 + DELTA_T) * current * (flow + MIN_FLOW);

                vehicleConductivity[vehicleIdx][i][j] = newConductivity;
                vehicleConductivity[vehicleIdx][j][i] = newConductivity;
            }
        }
    }

    // Invalidar cache após atualização
    fill(cacheValid[vehicleIdx].begin(), cacheValid[vehicleIdx].end(), false);
}

bool PhysarumSolver::isRouteFeasible(const vector<int>& route) const {
    double totalDemand = 0.0;
    for (int node : route) {
        if (node != depot) {
            auto it = demands.find(node);
            if (it != demands.end()) {
                totalDemand += it->second;
            }
        }
    }
    return totalDemand <= vehicleCapacity;
}

double PhysarumSolver::calculateRouteSegmentDistance(int from, int to) const {
    return adjacencyMatrix[from][to];
}

vector<Route> PhysarumSolver::findRoutes() {
    // Evolui a rede primeiro
    evolveNetwork();

    vector<Route> routes;
    set<int> nodesToVisit;

    for (int i = 0; i < numNodes; ++i) {
        if (i != depot) nodesToVisit.insert(i);
    }

    while (!nodesToVisit.empty() && routes.size() < static_cast<size_t>(numVehicles)) {
        Route currentRoute;
        currentRoute.nodes.push_back(depot);
        double currentDemand = 0.0;

        while (!nodesToVisit.empty()) {
            int currentNode = currentRoute.nodes.back();

            struct CandidateNode {
                int node;
                double conductivityScore;
                double distance;
            };
            vector<CandidateNode> candidates;

            // Avalia candidatos usando condutividade global e local
            for (int node : nodesToVisit) {
                double potentialDemand = currentDemand + demands[node];
                if (potentialDemand <= vehicleCapacity) {
                    double distance = adjacencyMatrix[currentNode][node];
                    if (distance > 0) {
                        // Combina scores globais e locais
                        double globalScore = globalConductivityScores[currentNode][node];
                        double localScore = 0;
                        for (int v = 0; v < numVehicles; v++)
                            localScore += calculateNodeScore(currentNode, node, v);
                        localScore /= numVehicles;

                        double combinedScore = GLOBAL_PRESSURE_WEIGHT * globalScore +
                                               (1 - GLOBAL_PRESSURE_WEIGHT) * localScore;

                        candidates.push_back({node, combinedScore, distance});
                    }
                }
            }

            if (candidates.empty()) break;

            // Ordena por score combinado
            sort(candidates.begin(), candidates.end(),
                 [](const CandidateNode& a, const CandidateNode& b) {
                     return a.conductivityScore > b.conductivityScore;
                 });

            int nextNode = candidates[0].node;
            double actualDistance = candidates[0].distance;

            currentRoute.nodes.push_back(nextNode);
            currentDemand += demands[nextNode];
            currentRoute.totalDemand = currentDemand;
            currentRoute.totalDistance += actualDistance;
            nodesToVisit.erase(nextNode);

            // Atualiza condutividades locais
            for (int vehicleIdx = 0; vehicleIdx < numVehicles; ++vehicleIdx) {
                auto pressures = calculatePressures(currentRoute.nodes.front(),
                                                    currentRoute.nodes.back(),
                                                    vehicleIdx);

                vector<vector<double>> flux(numNodes, vector<double>(numNodes, 0.0));
                for (int i = 0; i < numNodes; ++i) {
                    for (int j = 0; j < numNodes; ++j) {
                        if (adjacencyMatrix[i][j] > 0) {
                            flux[i][j] = vehicleConductivity[vehicleIdx][i][j] *
                                         abs(pressures[i] - pressures[j]);
                        }
                    }
                }
                updateConductivity(flux, vehicleIdx);
            }
        }

        double finalDistance = calculateRouteSegmentDistance(currentRoute.nodes.back(), depot);
        currentRoute.totalDistance += finalDistance;
        currentRoute.nodes.push_back(depot);

        routes.push_back(currentRoute);
    }

    return routes;
}

void PhysarumSolver::evolveNetwork() {
    vector<vector<double>> oldConductivity;
    globalPressures.resize(numNodes, vector<double>(numNodes));
    globalConductivityScores.resize(numNodes, vector<double>(numNodes));

    for (int iter = 0; iter < NETWORK_EVOLUTION_ITERATIONS; iter++) {
        oldConductivity = conductivity;

        // Calcula pressões globais
        auto pressures = calculateGlobalPressures();

        // Atualiza condutividades
        updateGlobalConductivities(pressures);

        if (hasNetworkConverged(oldConductivity)) {
            cout << "Rede convergiu em " << iter << " iterações\n";
            break;
        }
    }
}

vector<vector<double>> PhysarumSolver::calculateGlobalPressures() {
    vector<vector<double>> pressures(numNodes, vector<double>(numNodes));

#pragma omp parallel for collapse(2)
    for (int i = 0; i < numNodes; i++) {
        for (int j = 0; j < numNodes; j++) {
            if (i != j && adjacencyMatrix[i][j] > 0) {
                auto pathPressures = calculatePressures(i, j, 0);
                pressures[i][j] = pathPressures[j];
            }
        }
    }

    return pressures;
}

double PhysarumSolver::evaluateConnectionQuality(int from, int to, double demand) {
    double distance = adjacencyMatrix[from][to];
    double remainingCapacity = vehicleCapacity - demand;

    // Penaliza conexões que deixam pouca capacidade residual
    double capacityFactor = remainingCapacity / vehicleCapacity;

    // Considera a média das condutividades de todos os veículos
    double avgConductivity = 0.0;
    for(int v = 0; v < numVehicles; v++) {
        avgConductivity += vehicleConductivity[v][from][to];
    }
    avgConductivity /= numVehicles;

    // Nova fórmula de score que considera mais fatores
    return (avgConductivity * MU + capacityFactor * 2.0) /
           (pow(distance, 2.0) * (1.0 + abs(demands[to] - demands[from])/vehicleCapacity));
}

void PhysarumSolver::updateGlobalConductivities(const vector<vector<double>>& pressures) {

    if (conductivity == vector<vector<double>>(numNodes, vector<double>(numNodes, 1.0))) {
        for (int i = 0; i < numNodes; i++) {
            for (int j = 0; j < numNodes; j++) {
                if (adjacencyMatrix[i][j] > 0) {
                    conductivity[i][j] *= (1.0 + ((rand() % 100) / 1000.0));
                }
            }
        }
    }

    vector<vector<double>> flowMatrix(numNodes, vector<double>(numNodes, 0.0));

    // Calcula fluxos totais primeiro
    for (int i = 0; i < numNodes; i++) {
        for (int j = 0; j < numNodes; j++) {
            if (adjacencyMatrix[i][j] > 0) {
                flowMatrix[i][j] = calculateGlobalFlow(i, j, pressures);
            }
        }
    }

    // Encontra fluxo máximo para normalização
    double maxFlow = 0.0;
    for (const auto& row : flowMatrix) {
        maxFlow = max(maxFlow, *max_element(row.begin(), row.end()));
    }

    // Atualiza condutividades com normalização
    for (int i = 0; i < numNodes; i++) {
        for (int j = 0; j < numNodes; j++) {
            if (adjacencyMatrix[i][j] > 0) {
                double normalizedFlow = flowMatrix[i][j] / maxFlow;
                double adaptiveDeltaT = DELTA_T * (1.0 + normalizedFlow);

                double newConductivity = (1 + adaptiveDeltaT) * conductivity[i][j] *
                                         (normalizedFlow + MIN_FLOW);

                // Adiciona um fator de momentum
                newConductivity = 0.85 * conductivity[i][j] + 0.15 * newConductivity;

                conductivity[i][j] = newConductivity;
                conductivity[j][i] = newConductivity;
            }
        }
    }
}

double PhysarumSolver::calculateGlobalFlow(int from, int to,
                                           const vector<vector<double>>& pressures) {
    return conductivity[from][to] * abs(pressures[from][to] - pressures[to][from]);
}

bool PhysarumSolver::hasNetworkConverged(const vector<vector<double>>& oldConductivity) {
    double maxDiff = 0.0;

#pragma omp parallel for collapse(2) reduction(max:maxDiff)
    for (int i = 0; i < numNodes; i++) {
        for (int j = 0; j < numNodes; j++) {
            if (adjacencyMatrix[i][j] > 0) {
                maxDiff = max(maxDiff,
                              abs(conductivity[i][j] - oldConductivity[i][j]));
            }
        }
    }

    return maxDiff < EPSILON;
}

void PhysarumSolver::printSolution(const vector<Route>& routes) {
    cout << "\nRotas encontradas:\n";
    for (size_t i = 0; i < routes.size(); ++i) {
        cout << "\nRota " << (i + 1) << ":\n";
        cout << "Sequência: ";
        for (size_t j = 0; j < routes[i].nodes.size(); ++j) {
            cout << routes[i].nodes[j];
            if (j < routes[i].nodes.size() - 1) cout << " -> ";
        }
        cout << "\nDemanda total: " << routes[i].totalDemand;
        cout << "\nDistância total: " << routes[i].totalDistance << "\n";
    }
}

double PhysarumSolver::calculateEuclideanDistance(const Point& p1, const Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return round(sqrt(dx * dx + dy * dy));
}


vector<pair<int, int>> PhysarumSolver::findMainPaths() {
    vector<pair<int, int>> mainPaths;
    vector<bool> visited(numNodes, false);
    visited[depot] = true;

    // Encontra caminhos com maior condutividade
    for (int v = 0; v < numVehicles; v++) {
        int current = depot;
        vector<int> path;
        path.push_back(current);

        while (true) {
            double maxConductivity = -1;
            int nextNode = -1;

            for (int j = 0; j < numNodes; j++) {
                if (!visited[j] && adjacencyMatrix[current][j] > 0 &&
                    conductivity[current][j] > maxConductivity) {
                    maxConductivity = conductivity[current][j];
                    nextNode = j;
                }
            }

            if (nextNode == -1) break;

            path.push_back(nextNode);
            visited[nextNode] = true;
            current = nextNode;
        }

        if (path.size() > 1) {
            mainPaths.push_back({path.front(), path.back()});
        }
    }

    return mainPaths;
}

Route PhysarumSolver::constructRouteFromPath(const pair<int, int>& path) {
    Route route;
    route.nodes.push_back(depot);

    // Reconstrói caminho usando condutividades
    int current = path.first;
    set<int> visited;
    visited.insert(depot);

    while (current != depot && route.totalDemand < vehicleCapacity) {
        if (visited.find(current) == visited.end()) {
            route.nodes.push_back(current);
            route.totalDemand += demands[current];
            visited.insert(current);
        }

        // Encontra próximo nó com maior condutividade
        double maxConductivity = -1;
        int nextNode = depot;

        for (int j = 0; j < numNodes; j++) {
            if (adjacencyMatrix[current][j] > 0 &&
                conductivity[current][j] > maxConductivity) {
                maxConductivity = conductivity[current][j];
                nextNode = j;
            }
        }

        current = nextNode;
    }

    route.nodes.push_back(depot);
    return route;
}