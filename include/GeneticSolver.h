#ifndef GENETIC_SOLVER_H
#define GENETIC_SOLVER_H

#include "PhysarumSolver.h"
#include <vector>
#include <random>
#include <functional>

// Representa um indivíduo na população genética
struct Individual {
    std::vector<Route> routes;   // Solução (conjunto de rotas)
    double fitness;              // Valor de fitness (inverso da distância total)
    double totalDistance;        // Distância total da solução

    Individual() : fitness(0), totalDistance(0) {}

    // Construtor a partir de rotas existentes
    Individual(const std::vector<Route>& r) : routes(r), fitness(0), totalDistance(0) {
        calculateTotalDistance();
        calculateFitness();
    }

    // Calcula distância total
    void calculateTotalDistance() {
        totalDistance = 0;
        for (const auto& route : routes) {
            totalDistance += route.totalDistance;
        }
    }

    // Calcula fitness (inverso da distância)
    void calculateFitness() {
        fitness = (totalDistance > 0) ? 1.0 / totalDistance : 0;
    }

    // Operador de comparação para ordenação
    bool operator<(const Individual& other) const {
        return fitness > other.fitness;  // Maior fitness = melhor indivíduo
    }
};

class GeneticSolver : public PhysarumSolver {
private:
    // Parâmetros do algoritmo genético
    int populationSize;
    int maxGenerations;
    double crossoverRate;
    double mutationRate;
    double elitismRate;

    // Gerador de números aleatórios
    std::mt19937 rng;
    std::uniform_real_distribution<double> uniformDist;

    // População atual
    std::vector<Individual> population;

    // Métodos privados
    void initializePopulation();
    std::vector<Individual> selection(const std::vector<Individual>& currentPopulation);
    Individual crossover(const Individual& parent1, const Individual& parent2);
    void mutate(Individual& individual);
    bool isRouteFeasible(const Route& route) const;
    double calculateRouteDistance(const Route& route) const;
    void updateRouteDistances(Individual& individual);
    bool validateSolution(const Individual& individual) const;
    int selectRouteForCrossover(const Individual& parent, std::vector<bool>& selectedNodes) const;
    void repairSolution(Individual& offspring) const;

    // Operadores de mutação específicos para CVRP
    void swapNodesMutation(Route& route);
    void insertNodeMutation(Route& route);
    void reverseMutation(Route& route);
    void relocateNodeBetweenRoutesMutation(Individual& individual);
    void swapNodesBetweenRoutesMutation(Individual& individual);
    void routeMergeMutation(Individual& individual);
    void routeSplitMutation(Individual& individual);

public:
    GeneticSolver(int nodes, int vehicles, double capacity, int depotNode,
                  int popSize = 50, int maxGen = 100,
                  double cRate = 0.8, double mRate = 0.2, double eRate = 0.1);

    // Sobrescreve o método findRoutes para usar algoritmo genético
    std::vector<Route> findRoutes() override;

    // Métodos para debug e análise
    void printPopulationStats() const;
    Individual getBestIndividual() const;
    void printEvolutionStats(int generation, double bestDistance, double avgDistance, double worstDistance) const;
};

#endif // GENETIC_SOLVER_H