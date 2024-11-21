#include "../include/PhysarumSolver.h"
#include <iostream>
#include <set>
#include <limits>
#include <cmath>
#include <iomanip>
#include <algorithm>

using namespace std;

// Constantes como membros estáticos
const int MAX_ITERATIONS = 100;
const double MU = 1.3;
const double DELTA_T = 0.01;
const double EPSILON = 1e-6;
const double MIN_FLOW = 0.01;

PhysarumSolver::PhysarumSolver(int nodes, int vehicles, double capacity, int depotNode)
    : numNodes(nodes), numVehicles(vehicles), vehicleCapacity(capacity), depot(depotNode) {
    
    adjacencyMatrix.resize(numNodes, vector<double>(numNodes, 0.0));
    conductivity.resize(numNodes, vector<double>(numNodes, 1.0));
    vehicleConductivity.resize(numVehicles,
        vector<vector<double>>(numNodes, vector<double>(numNodes, 1.0)));
    
    // Pré-alocar o cache de pressões
    pressureCache.resize(numVehicles);
    for (auto& cache : pressureCache) {
        cache.resize(numNodes, vector<double>(numNodes, 0.0));
    }
    cacheValid.resize(numVehicles, vector<bool>(numNodes, false));
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
    // Verificar cache
    if (cacheValid[vehicleIdx][source] && 
        !pressureCache[vehicleIdx][source].empty()) {
        return pressureCache[vehicleIdx][source];
    }

    vector<double> pressures(numNodes, 0.0);
    vector<double> previousPressures(numNodes);
    pressures[source] = 1.0;
    pressures[target] = 0.0;

    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        previousPressures = pressures;
        bool converged = true;

        for (int i = 0; i < numNodes; ++i) {
            if (i == source || i == target) continue;

            double sumConductivity = 0.0;
            double sumPressureFlow = 0.0;

            for (int j = 0; j < numNodes; ++j) {
                if (adjacencyMatrix[i][j] > 0) {
                    sumConductivity += vehicleConductivity[vehicleIdx][i][j];
                    sumPressureFlow += vehicleConductivity[vehicleIdx][i][j] * pressures[j];
                }
            }

            if (sumConductivity > 0) {
                double newPressure = sumPressureFlow / sumConductivity;
                if (abs(newPressure - previousPressures[i]) > EPSILON) {
                    converged = false;
                }
                pressures[i] = newPressure;
            }
        }

        if (converged) break;
    }

    // Atualizar cache
    pressureCache[vehicleIdx][source] = pressures;
    cacheValid[vehicleIdx][source] = true;

    return pressures;
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

vector<Route> PhysarumSolver::findRoutes() {
    vector<Route> routes;
    set<int> nodesToVisit;

    // Inicialização
    for (int i = 0; i < numNodes; ++i) {
        if (i != depot) nodesToVisit.insert(i);
    }

    while (!nodesToVisit.empty() && routes.size() < numVehicles) {
        Route currentRoute;
        currentRoute.nodes.push_back(depot);
        double currentDemand = 0.0;

        while (!nodesToVisit.empty()) {
            int currentNode = currentRoute.nodes.back();
            
            // Estrutura para armazenar candidatos viáveis
            vector<pair<double, int>> candidates;
            
            // Pré-calcular candidatos viáveis
            for (int node : nodesToVisit) {
                double potentialDemand = currentDemand + demands[node];
                if (potentialDemand <= vehicleCapacity) {
                    double distance = adjacencyMatrix[currentNode][node];
                    if (distance > 0) {
                        candidates.emplace_back(distance, node);
                    }
                }
            }

            if (candidates.empty()) break;

            // Ordenar candidatos por distância
            sort(candidates.begin(), candidates.end());
            
            int nextNode = candidates[0].second;
            double minDistance = candidates[0].first;

            currentRoute.nodes.push_back(nextNode);
            currentDemand += demands[nextNode];
            currentRoute.totalDemand = currentDemand;
            currentRoute.totalDistance += minDistance;
            nodesToVisit.erase(nextNode);

            // Atualizar conductvidades
            for (int vehicleIdx = 0; vehicleIdx < numVehicles; ++vehicleIdx) {
                auto pressures = calculatePressures(currentRoute.nodes.front(),
                                                  currentRoute.nodes.back(),
                                                  vehicleIdx);

                vector<vector<double>> flux(numNodes, vector<double>(numNodes, 0.0));
                
                // Calcular fluxo
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

        currentRoute.nodes.push_back(depot);
        routes.push_back(currentRoute);
    }

    return routes;
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