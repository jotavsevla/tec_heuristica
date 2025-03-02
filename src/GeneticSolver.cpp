#include "../include/GeneticSolver.h"
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <set>
#include <cassert>
#include <cmath>

using namespace std;

GeneticSolver::GeneticSolver(int nodes, int vehicles, double capacity, int depotNode,
                             int popSize, int maxGen, double cRate, double mRate, double eRate)
        : PhysarumSolver(nodes, vehicles, capacity, depotNode),
          populationSize(popSize),
          maxGenerations(maxGen),
          crossoverRate(cRate),
          mutationRate(mRate),
          elitismRate(eRate) {

    // Inicializa o gerador de números aleatórios com uma semente
    std::random_device rd;
    rng = std::mt19937(rd());
    uniformDist = std::uniform_real_distribution<double>(0.0, 1.0);
}

void GeneticSolver::initializePopulation() {
    cout << "Inicializando população com " << populationSize << " indivíduos..." << endl;
    population.clear();

    // Primeiro indivíduo: solução do Physarum original
    auto physarumSolution = PhysarumSolver::findRoutes();
    population.push_back(Individual(physarumSolution));

    // Gera variações da solução do Physarum para o resto da população
    while (population.size() < static_cast<size_t>(populationSize)) {
        Individual newIndividual = population[0];  // Cópia do indivíduo base

        // Aplica mutações aleatórias para diversificar
        for (int i = 0; i < 5; i++) {  // Aplica várias mutações para diversificar
            mutate(newIndividual);
        }

        // Verifica se a solução é válida antes de adicionar
        if (validateSolution(newIndividual)) {
            updateRouteDistances(newIndividual);
            newIndividual.calculateTotalDistance();
            newIndividual.calculateFitness();
            population.push_back(newIndividual);
        }
    }

    // Ordena a população pelo fitness
    sort(population.begin(), population.end());
}

vector<Individual> GeneticSolver::selection(const vector<Individual>& currentPopulation) {
    vector<Individual> selected;
    int populationCount = currentPopulation.size();

    // Elitismo: mantém os melhores indivíduos
    int eliteCount = static_cast<int>(elitismRate * populationCount);
    for (int i = 0; i < eliteCount && i < populationCount; i++) {
        selected.push_back(currentPopulation[i]);
    }

    // Seleção por torneio
    while (selected.size() < static_cast<size_t>(populationCount)) {
        // Seleciona 3 indivíduos aleatoriamente
        uniform_int_distribution<int> dist(0, populationCount - 1);
        int idx1 = dist(rng);
        int idx2 = dist(rng);
        int idx3 = dist(rng);

        // Seleciona o melhor dos três
        int bestIdx = idx1;
        if (currentPopulation[idx2].fitness > currentPopulation[bestIdx].fitness) {
            bestIdx = idx2;
        }
        if (currentPopulation[idx3].fitness > currentPopulation[bestIdx].fitness) {
            bestIdx = idx3;
        }

        selected.push_back(currentPopulation[bestIdx]);
    }

    return selected;
}

int GeneticSolver::selectRouteForCrossover(const Individual& parent, vector<bool>& selectedNodes)  {
    if (parent.routes.empty()) {
        return -1;
    }

    // Tenta encontrar uma rota que contenha nós ainda não selecionados
    vector<int> candidateRoutes;

    for (size_t i = 0; i < parent.routes.size(); i++) {
        const Route& route = parent.routes[i];
        bool hasUnselectedNodes = false;

        for (size_t j = 1; j < route.nodes.size() - 1; j++) {
            int node = route.nodes[j];
            if (!selectedNodes[node]) {
                hasUnselectedNodes = true;
                break;
            }
        }

        if (hasUnselectedNodes) {
            candidateRoutes.push_back(i);
        }
    }

    if (candidateRoutes.empty()) {
        return -1;
    }

    // Seleciona uma rota aleatória dentre as candidatas
    uniform_int_distribution<int> dist(0, candidateRoutes.size() - 1);
    return candidateRoutes[dist(rng)];
}

Individual GeneticSolver::crossover(const Individual& parent1, const Individual& parent2) {
    if (uniformDist(rng) > crossoverRate) {
        // Se não fizer crossover, retorna uma cópia de um dos pais
        return (uniformDist(rng) < 0.5) ? parent1 : parent2;
    }

    Individual offspring;
    vector<bool> selectedNodes(numNodes, false);
    selectedNodes[depot] = true;  // O depósito é sempre considerado selecionado

    // Inicializa com rotas vazias
    offspring.routes.clear();

    // Seleciona rotas alternadamente dos pais
    while (true) {
        int routeIdxP1 = selectRouteForCrossover(parent1, selectedNodes);
        int routeIdxP2 = selectRouteForCrossover(parent2, selectedNodes);

        if (routeIdxP1 == -1 && routeIdxP2 == -1) {
            break;  // Não há mais rotas disponíveis em ambos os pais
        }

        if (routeIdxP1 != -1) {
            // Adiciona uma rota do parent1
            Route newRoute = parent1.routes[routeIdxP1];
            offspring.routes.push_back(newRoute);

            // Marca os nós como selecionados
            for (int node : newRoute.nodes) {
                selectedNodes[node] = true;
            }
        }

        if (routeIdxP2 != -1) {
            // Adiciona uma rota do parent2
            Route newRoute = parent2.routes[routeIdxP2];
            offspring.routes.push_back(newRoute);

            // Marca os nós como selecionados
            for (int node : newRoute.nodes) {
                selectedNodes[node] = true;
            }
        }
    }

    // Repara a solução se necessário
    repairSolution(offspring);

    // Atualiza distâncias e fitness
    updateRouteDistances(offspring);
    offspring.calculateTotalDistance();
    offspring.calculateFitness();

    return offspring;
}

void GeneticSolver::repairSolution(Individual& offspring) const {
    // Verifica se todos os nós estão presentes
    vector<bool> visitedNodes(numNodes, false);
    visitedNodes[depot] = true;  // O depósito é sempre visitado

    for (const Route& route : offspring.routes) {
        for (size_t j = 1; j < route.nodes.size() - 1; j++) {
            int node = route.nodes[j];
            visitedNodes[node] = true;
        }
    }

    // Encontra nós ausentes
    vector<int> missingNodes;
    for (int i = 0; i < numNodes; i++) {
        if (!visitedNodes[i] && i != depot) {
            missingNodes.push_back(i);
        }
    }

    // Verifica nós duplicados
    vector<int> nodeCount(numNodes, 0);
    for (const Route& route : offspring.routes) {
        for (size_t j = 1; j < route.nodes.size() - 1; j++) {
            int node = route.nodes[j];
            nodeCount[node]++;
        }
    }

    vector<int> duplicateNodes;
    for (int i = 0; i < numNodes; i++) {
        if (nodeCount[i] > 1 && i != depot) {
            duplicateNodes.push_back(i);
        }
    }

    // Remover nós duplicados, deixando apenas 1 ocorrência
    for (int dupNode : duplicateNodes) {
        bool found = false;
        for (Route& route : offspring.routes) {
            for (size_t j = 1; j < route.nodes.size() - 1; j++) {
                if (route.nodes[j] == dupNode && found) {
                    route.nodes.erase(route.nodes.begin() + j);
                    j--;  // Ajusta o índice após remoção
                } else if (route.nodes[j] == dupNode) {
                    found = true;
                }
            }
        }
    }

    // Recalcula demandas após remover duplicatas
    for (Route& route : offspring.routes) {
        route.totalDemand = 0;
        for (size_t j = 1; j < route.nodes.size() - 1; j++) {
            int node = route.nodes[j];
            auto it = demands.find(node);
            if (it != demands.end()) {
                route.totalDemand += it->second;
            }
        }
    }

    // Adicionar nós ausentes
    for (int missingNode : missingNodes) {
        // Procura a melhor rota para inserir o nó
        double bestCost = numeric_limits<double>::max();
        size_t bestRouteIdx = 0;
        size_t bestPos = 0;

        auto demandIt = demands.find(missingNode);
        double nodeDemand = (demandIt != demands.end()) ? demandIt->second : 0;

        for (size_t i = 0; i < offspring.routes.size(); i++) {
            Route& route = offspring.routes[i];

            // Verifica se a rota pode receber o nó em termos de capacidade
            if (route.totalDemand + nodeDemand > vehicleCapacity) {
                continue;
            }

            for (size_t j = 1; j < route.nodes.size(); j++) {
                double currentCost = calculateRouteSegmentDistance(route.nodes[j-1], route.nodes[j]);
                double newCost = calculateRouteSegmentDistance(route.nodes[j-1], missingNode) +
                                 calculateRouteSegmentDistance(missingNode, route.nodes[j]);
                double deltaCost = newCost - currentCost;

                if (deltaCost < bestCost) {
                    bestCost = deltaCost;
                    bestRouteIdx = i;
                    bestPos = j;
                }
            }
        }

        // Se não encontrar nenhuma rota viável, cria uma nova
        if (bestCost == numeric_limits<double>::max()) {
            Route newRoute;
            newRoute.nodes.push_back(depot);
            newRoute.nodes.push_back(missingNode);
            newRoute.nodes.push_back(depot);
            newRoute.totalDemand = nodeDemand;
            offspring.routes.push_back(newRoute);
        } else {
            // Insere o nó na melhor posição
            Route& bestRoute = offspring.routes[bestRouteIdx];
            bestRoute.nodes.insert(bestRoute.nodes.begin() + bestPos, missingNode);
            bestRoute.totalDemand += nodeDemand;
        }
    }

    // Remove rotas vazias
    offspring.routes.erase(
            remove_if(offspring.routes.begin(), offspring.routes.end(),
                      [this](const Route& route) {
                          return route.nodes.size() <= 2;  // Só tem depósito no início e fim
                      }),
            offspring.routes.end()
    );
}

void GeneticSolver::mutate(Individual& individual) {
    if (uniformDist(rng) > mutationRate) {
        return;  // Não faz mutação
    }

    // Escolhe aleatoriamente um tipo de mutação
    int mutationType = uniform_int_distribution<int>(0, 6)(rng);

    switch (mutationType) {
        case 0:  // Swap nodes dentro de uma única rota
            if (!individual.routes.empty()) {
                uniform_int_distribution<size_t> routeDist(0, individual.routes.size() - 1);
                swapNodesMutation(individual.routes[routeDist(rng)]);
            }
            break;

        case 1:  // Inserção de nó dentro de uma rota
            if (!individual.routes.empty()) {
                uniform_int_distribution<size_t> routeDist(0, individual.routes.size() - 1);
                insertNodeMutation(individual.routes[routeDist(rng)]);
            }
            break;

        case 2:  // Reversão de segmento dentro de uma rota
            if (!individual.routes.empty()) {
                uniform_int_distribution<size_t> routeDist(0, individual.routes.size() - 1);
                reverseMutation(individual.routes[routeDist(rng)]);
            }
            break;

        case 3:  // Realocação de nó entre rotas
            relocateNodeBetweenRoutesMutation(individual);
            break;

        case 4:  // Swap de nós entre rotas
            swapNodesBetweenRoutesMutation(individual);
            break;

        case 5:  // Merge de rotas (possível se não exceder capacidade)
            routeMergeMutation(individual);
            break;

        case 6:  // Split de rota
            routeSplitMutation(individual);
            break;
    }

    // Atualiza as distâncias e fitness
    updateRouteDistances(individual);
    individual.calculateTotalDistance();
    individual.calculateFitness();
}

void GeneticSolver::swapNodesMutation(Route& route) {
    // Número de nós na rota (excluindo depósito no início e fim)
    int routeSize = route.nodes.size() - 2;
    if (routeSize < 2) return;  // Precisa de pelo menos 2 nós para swap

    // Seleciona dois índices diferentes
    uniform_int_distribution<int> dist(1, routeSize);
    int idx1 = dist(rng);
    int idx2;
    do {
        idx2 = dist(rng);
    } while (idx2 == idx1);

    // Swap dos nós
    swap(route.nodes[idx1], route.nodes[idx2]);
}

void GeneticSolver::insertNodeMutation(Route& route) {
    // Número de nós na rota (excluindo depósito no início e fim)
    int routeSize = route.nodes.size() - 2;
    if (routeSize < 2) return;  // Precisa de pelo menos 2 nós

    // Seleciona um nó para remover e um local para inserir
    uniform_int_distribution<int> dist(1, routeSize);
    int removeIdx = dist(rng);
    int insertIdx;
    do {
        insertIdx = dist(rng);
    } while (insertIdx == removeIdx);

    // Remove o nó e o insere em outra posição
    int node = route.nodes[removeIdx];
    route.nodes.erase(route.nodes.begin() + removeIdx);

    // Ajusta o índice de inserção se necessário
    if (insertIdx > removeIdx) {
        insertIdx--;
    }

    route.nodes.insert(route.nodes.begin() + insertIdx, node);
}

void GeneticSolver::reverseMutation(Route& route) {
    // Número de nós na rota (excluindo depósito no início e fim)
    int routeSize = route.nodes.size() - 2;
    if (routeSize < 2) return;  // Precisa de pelo menos 2 nós

    // Seleciona um segmento aleatório para reverter
    uniform_int_distribution<int> dist(1, routeSize);
    int start = dist(rng);
    int end = dist(rng);

    if (start > end) {
        swap(start, end);
    }

    // Reverte o segmento
    reverse(route.nodes.begin() + start, route.nodes.begin() + end + 1);
}

void GeneticSolver::relocateNodeBetweenRoutesMutation(Individual& individual) {
    if (individual.routes.size() < 2) return;  // Precisa de pelo menos 2 rotas

    // Identifica pares viáveis de rotas para relocação
    vector<tuple<size_t, size_t, int, double>> viableMoves; // (rotaOrigem, rotaDestino, índiceNó, distânciaGanho)

    // Analisa todas as rotas de origem possíveis
    for (size_t sourceIdx = 0; sourceIdx < individual.routes.size(); sourceIdx++) {
        Route& sourceRoute = individual.routes[sourceIdx];

        // Pula rotas com poucos nós
        if (sourceRoute.nodes.size() <= 3) continue;

        // Para cada nó na rota de origem
        for (size_t nodeIdx = 1; nodeIdx < sourceRoute.nodes.size() - 1; nodeIdx++) {
            int node = sourceRoute.nodes[nodeIdx];
            auto demandIt = demands.find(node);
            double nodeDemand = (demandIt != demands.end()) ? demandIt->second : 0;

            // Calcula o ganho de distância se este nó for removido da rota
            int prevNode = sourceRoute.nodes[nodeIdx - 1];
            int nextNode = sourceRoute.nodes[nodeIdx + 1];
            double currentDistance = calculateRouteSegmentDistance(prevNode, node) +
                                     calculateRouteSegmentDistance(node, nextNode);
            double newDistance = calculateRouteSegmentDistance(prevNode, nextNode);
            double distanceGain = currentDistance - newDistance;

            // Analisa todas as rotas de destino possíveis
            for (size_t targetIdx = 0; targetIdx < individual.routes.size(); targetIdx++) {
                if (sourceIdx == targetIdx) continue;

                Route& targetRoute = individual.routes[targetIdx];

                // Verifica se a capacidade permite adicionar este nó
                if (targetRoute.totalDemand + nodeDemand > vehicleCapacity) {
                    continue;
                }

                // Encontra a melhor posição para inserir na rota de destino
                double bestInsertionCost = numeric_limits<double>::max();
                int bestInsertPos = -1;

                for (size_t insertPos = 1; insertPos < targetRoute.nodes.size(); insertPos++) {
                    int prev = targetRoute.nodes[insertPos - 1];
                    int next = targetRoute.nodes[insertPos];

                    double currentSegmentDist = calculateRouteSegmentDistance(prev, next);
                    double newSegmentDist = calculateRouteSegmentDistance(prev, node) +
                                            calculateRouteSegmentDistance(node, next);
                    double insertionCost = newSegmentDist - currentSegmentDist;

                    if (insertionCost < bestInsertionCost) {
                        bestInsertionCost = insertionCost;
                        bestInsertPos = insertPos;
                    }
                }

                // Calcula o ganho total considerando remoção e inserção
                double totalGain = distanceGain - bestInsertionCost;

                // Adiciona à lista de movimentos viáveis
                viableMoves.push_back({sourceIdx, targetIdx, nodeIdx, totalGain});
            }
        }
    }

    // Se não há movimentos viáveis, não faz nada
    if (viableMoves.empty()) return;

    // Opção 1: Escolhe um movimento aleatório dentre os viáveis
    // uniform_int_distribution<size_t> moveDist(0, viableMoves.size() - 1);
    // auto [sourceRouteIdx, targetRouteIdx, nodeIdx, gain] = viableMoves[moveDist(rng)];

    // Opção 2: Escolhe o movimento com maior ganho de distância (menor custo)
    sort(viableMoves.begin(), viableMoves.end(),
         [](const auto& a, const auto& b) { return get<3>(a) > get<3>(b); });

    // Probabilidade de escolher os melhores movimentos (topo 20%)
    if (uniformDist(rng) < 0.8 && viableMoves.size() >= 5) {
        uniform_int_distribution<size_t> topMoveDist(0, viableMoves.size() / 5);
        auto [sourceRouteIdx, targetRouteIdx, nodeIdx, gain] = viableMoves[topMoveDist(rng)];

        // Executa o movimento
        Route& sourceRoute = individual.routes[sourceRouteIdx];
        Route& targetRoute = individual.routes[targetRouteIdx];

        int node = sourceRoute.nodes[nodeIdx];
        auto demandIt = demands.find(node);
        double nodeDemand = (demandIt != demands.end()) ? demandIt->second : 0;

        // Remove o nó da rota de origem
        sourceRoute.nodes.erase(sourceRoute.nodes.begin() + nodeIdx);
        sourceRoute.totalDemand -= nodeDemand;

        // Encontra a melhor posição para inserir na rota de destino
        double bestInsertionCost = numeric_limits<double>::max();
        int bestInsertPos = 1; // padrão se não encontrar melhor

        for (size_t insertPos = 1; insertPos < targetRoute.nodes.size(); insertPos++) {
            int prev = targetRoute.nodes[insertPos - 1];
            int next = targetRoute.nodes[insertPos];

            double currentSegmentDist = calculateRouteSegmentDistance(prev, next);
            double newSegmentDist = calculateRouteSegmentDistance(prev, node) +
                                    calculateRouteSegmentDistance(node, next);
            double insertionCost = newSegmentDist - currentSegmentDist;

            if (insertionCost < bestInsertionCost) {
                bestInsertionCost = insertionCost;
                bestInsertPos = insertPos;
            }
        }

        // Insere na melhor posição
        targetRoute.nodes.insert(targetRoute.nodes.begin() + bestInsertPos, node);
        targetRoute.totalDemand += nodeDemand;
    }
    else {
        // Escolhe um movimento aleatório para manter diversidade
        uniform_int_distribution<size_t> moveDist(0, viableMoves.size() - 1);
        auto [sourceRouteIdx, targetRouteIdx, nodeIdx, gain] = viableMoves[moveDist(rng)];

        Route& sourceRoute = individual.routes[sourceRouteIdx];
        Route& targetRoute = individual.routes[targetRouteIdx];

        int node = sourceRoute.nodes[nodeIdx];
        auto demandIt = demands.find(node);
        double nodeDemand = (demandIt != demands.end()) ? demandIt->second : 0;

        // Remove o nó da rota de origem
        sourceRoute.nodes.erase(sourceRoute.nodes.begin() + nodeIdx);
        sourceRoute.totalDemand -= nodeDemand;

        // Insere o nó na rota de destino em uma posição aleatória
        uniform_int_distribution<int> insertDist(1, targetRoute.nodes.size() - 1);
        int insertIdx = insertDist(rng);
        targetRoute.nodes.insert(targetRoute.nodes.begin() + insertIdx, node);
        targetRoute.totalDemand += nodeDemand;
    }
}

void GeneticSolver::swapNodesBetweenRoutesMutation(Individual& individual) {
    if (individual.routes.size() < 2) return;  // Precisa de pelo menos 2 rotas

    // Tenta encontrar pares de rotas viáveis para troca
    vector<pair<size_t, size_t>> candidatePairs;

    for (size_t i = 0; i < individual.routes.size(); i++) {
        for (size_t j = i+1; j < individual.routes.size(); j++) {
            Route& route1 = individual.routes[i];
            Route& route2 = individual.routes[j];

            // Ambas rotas precisam ter nós além dos depósitos
            if (route1.nodes.size() <= 3 || route2.nodes.size() <= 3) continue;

            // Verifica se poderia haver trocas viáveis entre estas rotas
            // considerando a demanda mínima e máxima dos nós em cada rota
            double minDemand1 = numeric_limits<double>::max();
            double maxDemand1 = 0;
            double minDemand2 = numeric_limits<double>::max();
            double maxDemand2 = 0;

            // Calcula valores min/max de demanda para cada rota
            for (size_t k = 1; k < route1.nodes.size() - 1; k++) {
                auto demandIt = demands.find(route1.nodes[k]);
                double nodeDemand = (demandIt != demands.end()) ? demandIt->second : 0;
                minDemand1 = min(minDemand1, nodeDemand);
                maxDemand1 = max(maxDemand1, nodeDemand);
            }

            for (size_t k = 1; k < route2.nodes.size() - 1; k++) {
                auto demandIt = demands.find(route2.nodes[k]);
                double nodeDemand = (demandIt != demands.end()) ? demandIt->second : 0;
                minDemand2 = min(minDemand2, nodeDemand);
                maxDemand2 = max(maxDemand2, nodeDemand);
            }

            // Verifica se uma troca de nós entre estas rotas poderia ser viável
            // no pior caso (trocando nó de menor demanda por nó de maior demanda)
            double worstCase1 = route1.totalDemand - minDemand1 + maxDemand2;
            double worstCase2 = route2.totalDemand - minDemand2 + maxDemand1;

            if (worstCase1 <= vehicleCapacity && worstCase2 <= vehicleCapacity) {
                candidatePairs.push_back({i, j});
            }
        }
    }

    // Se não encontrou pares viáveis, sai sem fazer mutação
    if (candidatePairs.empty()) return;

    // Seleciona um par aleatório dentre os candidatos
    uniform_int_distribution<size_t> pairDist(0, candidatePairs.size() - 1);
    auto selectedPair = candidatePairs[pairDist(rng)];

    Route& route1 = individual.routes[selectedPair.first];
    Route& route2 = individual.routes[selectedPair.second];

    // Identifica nós viáveis para troca na primeira rota
    vector<pair<int, double>> viableNodes1; // (índice, demanda)
    for (size_t i = 1; i < route1.nodes.size() - 1; i++) {
        int node = route1.nodes[i];
        auto demandIt = demands.find(node);
        double nodeDemand = (demandIt != demands.end()) ? demandIt->second : 0;
        viableNodes1.push_back({i, nodeDemand});
    }

    // Para cada nó na rota 1, encontra nós viáveis na rota 2 para troca
    bool foundValidSwap = false;
    int node1Idx = -1, node2Idx = -1;
    double demand1 = 0, demand2 = 0;

    // Embaralha os nós para tentar diferentes combinações
    shuffle(viableNodes1.begin(), viableNodes1.end(), rng);

    for (auto& [idx1, demandNode1] : viableNodes1) {
        for (size_t j = 1; j < route2.nodes.size() - 1; j++) {
            int node2 = route2.nodes[j];
            auto demandIt = demands.find(node2);
            double demandNode2 = (demandIt != demands.end()) ? demandIt->second : 0;

            // Verifica viabilidade da troca em termos de capacidade
            double newDemand1 = route1.totalDemand - demandNode1 + demandNode2;
            double newDemand2 = route2.totalDemand - demandNode2 + demandNode1;

            if (newDemand1 <= vehicleCapacity && newDemand2 <= vehicleCapacity) {
                node1Idx = idx1;
                node2Idx = j;
                demand1 = demandNode1;
                demand2 = demandNode2;
                foundValidSwap = true;
                break;
            }
        }
        if (foundValidSwap) break;
    }

    // Se não encontrou troca viável, não faz mutação
    if (!foundValidSwap) return;

    // Realiza a troca
    int node1 = route1.nodes[node1Idx];
    int node2 = route2.nodes[node2Idx];

    route1.nodes[node1Idx] = node2;
    route2.nodes[node2Idx] = node1;

    // Atualiza as demandas
    route1.totalDemand = route1.totalDemand - demand1 + demand2;
    route2.totalDemand = route2.totalDemand - demand2 + demand1;
}

void GeneticSolver::routeSplitMutation(Individual& individual) {
    // Só faz split se houver pelo menos uma rota com muitos nós
    vector<size_t> candidateRoutes;

    for (size_t i = 0; i < individual.routes.size(); i++) {
        if (individual.routes[i].nodes.size() >= 7) {  // Depósito + pelo menos 5 nós + depósito
            candidateRoutes.push_back(i);
        }
    }

    if (candidateRoutes.empty()) {
        return;
    }

    // Seleciona uma rota aleatória dentre as candidatas
    uniform_int_distribution<size_t> routeDist(0, candidateRoutes.size() - 1);
    size_t routeIdx = candidateRoutes[routeDist(rng)];
    Route& route = individual.routes[routeIdx];

    // Seleciona um ponto de divisão (evitando os depósitos)
    uniform_int_distribution<int> splitDist(2, route.nodes.size() - 2);
    int splitPoint = splitDist(rng);

    // Cria uma nova rota
    Route newRoute;
    newRoute.nodes.push_back(depot);  // Depósito inicial

    // Move nós da rota original para a nova rota
    double newRouteDemand = 0;
    for (size_t i = splitPoint; i < route.nodes.size() - 1; i++) {
        int node = route.nodes[i];
        newRoute.nodes.push_back(node);

        auto demandIt = demands.find(node);
        if (demandIt != demands.end()) {
            newRouteDemand += demandIt->second;
        }
    }

    newRoute.nodes.push_back(depot);  // Depósito final
    newRoute.totalDemand = newRouteDemand;

    // Atualiza a rota original
    route.nodes.erase(route.nodes.begin() + splitPoint, route.nodes.end() - 1);

    // Recalcula a demanda da rota original
    route.totalDemand = 0;
    for (size_t i = 1; i < route.nodes.size() - 1; i++) {
        auto demandIt = demands.find(route.nodes[i]);
        if (demandIt != demands.end()) {
            route.totalDemand += demandIt->second;
        }
    }

    // Adiciona a nova rota à solução
    individual.routes.push_back(newRoute);
}

void GeneticSolver::updateRouteDistances(Individual& individual) {
    for (Route& route : individual.routes) {
        route.totalDistance = 0;
        route.totalDemand = 0;

        // Recalcula tanto a distância quanto a demanda total
        for (size_t i = 0; i < route.nodes.size() - 1; i++) {
            route.totalDistance += calculateRouteSegmentDistance(route.nodes[i], route.nodes[i+1]);

            // Adiciona demanda (exceto para o depósito)
            if (i > 0 && route.nodes[i] != depot) {
                auto demandIt = demands.find(route.nodes[i]);
                if (demandIt != demands.end()) {
                    route.totalDemand += demandIt->second;
                }
            }
        }
    }
}

bool GeneticSolver::validateSolution(const Individual& individual) const {
    // Verifica se todas as rotas são viáveis e se todos os nós estão presentes
    vector<bool> visitedNodes(numNodes, false);
    visitedNodes[depot] = true;  // O depósito é sempre visitado

    for (const Route& route : individual.routes) {
        // Verifica se a rota começa e termina no depósito
        if (route.nodes.front() != depot || route.nodes.back() != depot) {
            return false;
        }

        // Calcula a demanda total da rota e verifica capacidade
        double routeDemand = 0;
        for (size_t i = 1; i < route.nodes.size() - 1; i++) {
            int node = route.nodes[i];

            // Verifica se o nó é válido
            if (node < 0 || node >= numNodes) {
                return false;
            }

            // Marca o nó como visitado
            visitedNodes[node] = true;

            // Adiciona demanda
            auto demandIt = demands.find(node);
            if (demandIt != demands.end()) {
                routeDemand += demandIt->second;
            }
        }

        // Verifica se a demanda excede a capacidade
        if (routeDemand > vehicleCapacity) {
            return false;
        }
    }

    // Verifica se todos os nós foram visitados
    for (int i = 0; i < numNodes; i++) {
        if (!visitedNodes[i] && i != depot) {
            return false;
        }
    }

    return true;
}

double GeneticSolver::calculateRouteDistance(const Route& route) const {
    double distance = 0;
    for (size_t i = 0; i < route.nodes.size() - 1; i++) {
        distance += calculateRouteSegmentDistance(route.nodes[i], route.nodes[i+1]);
    }
    return distance;
}

vector<Route> GeneticSolver::findRoutes() {
    cout << "\nIniciando algoritmo genético para CVRP..." << endl;

    // Inicializa população usando solução do Physarum como base
    initializePopulation();

    cout << "População inicial criada. Fitness do melhor indivíduo: "
         << population[0].fitness << " (distância: " << population[0].totalDistance << ")" << endl;

    // Variáveis para controle de convergência
    int generationsWithoutImprovement = 0;
    double bestDistanceEver = population[0].totalDistance;
    int convergenceThreshold = 50; // Número de gerações sem melhoria para considerar convergência

    // Executa gerações do algoritmo genético
    for (int generation = 0; generation < maxGenerations; generation++) {
        // Seleciona indivíduos para reprodução
        vector<Individual> selected = selection(population);

        // Cria nova população
        vector<Individual> newPopulation;

        // Mantém a elite diretamente
        int eliteCount = static_cast<int>(elitismRate * populationSize);
        for (int i = 0; i < eliteCount && i < static_cast<int>(population.size()); i++) {
            newPopulation.push_back(population[i]);
        }

        // Completa a população com crossover e mutação
        while (newPopulation.size() < static_cast<size_t>(populationSize)) {
            // Seleciona dois pais aleatoriamente
            uniform_int_distribution<int> dist(0, selected.size() - 1);
            int idx1 = dist(rng);
            int idx2;
            do {
                idx2 = dist(rng);
            } while (idx2 == idx1);

            // Crossover
            Individual offspring = crossover(selected[idx1], selected[idx2]);

            // Mutação
            mutate(offspring);

            // Verifica se a solução é válida antes de adicionar
            if (validateSolution(offspring)) {
                newPopulation.push_back(offspring);
            }
        }

        // Atualiza a população
        population = newPopulation;

        // Ordena pelo fitness
        sort(population.begin(), population.end());

        // Calcula estatísticas da população
        double totalDistance = 0;
        double bestDistance = population[0].totalDistance;
        double worstDistance = population.back().totalDistance;

        for (const auto& individual : population) {
            totalDistance += individual.totalDistance;
        }

        double avgDistance = totalDistance / population.size();

        // Verifica se houve melhoria na melhor solução
        if (bestDistance < bestDistanceEver) {
            bestDistanceEver = bestDistance;
            generationsWithoutImprovement = 0;
        } else {
            generationsWithoutImprovement++;
        }

        // Exibe estatísticas periodicamente
        if (generation % 10 == 0 || generation == maxGenerations - 1 ||
            generationsWithoutImprovement >= convergenceThreshold) {
            printEvolutionStats(generation, bestDistance, avgDistance, worstDistance);
        }

        // Aplica técnicas de diversificação se estiver convergindo
        if (generationsWithoutImprovement > convergenceThreshold / 2) {
            // Aumenta taxa de mutação temporariamente
            double originalMutationRate = mutationRate;
            mutationRate = min(0.8, mutationRate * 1.5);

            // Aplica mutações mais agressivas em parte da população (exceto elite)
            for (size_t i = eliteCount; i < population.size(); i++) {
                if (uniformDist(rng) < 0.3) { // 30% de chance para cada indivíduo
                    for (int j = 0; j < 3; j++) { // Aplica múltiplas mutações
                        mutate(population[i]);
                    }
                    updateRouteDistances(population[i]);
                    population[i].calculateTotalDistance();
                    population[i].calculateFitness();
                }
            }

            // Restaura taxa de mutação original
            mutationRate = originalMutationRate;

            // Re-ordena a população após as mutações
            sort(population.begin(), population.end());
        }

        // Interrompe se convergir
        if (generationsWithoutImprovement >= convergenceThreshold) {
            cout << "\nConvergência detectada após " << generation << " gerações!" << endl;
            break;
        }
    }

    // Retorna a melhor solução encontrada
    cout << "\nAlgoritmo genético concluído. Melhor solução encontrada:" << endl;
    cout << "Distância total: " << population[0].totalDistance << endl;
    cout << "Número de rotas: " << population[0].routes.size() << endl;

    // Aplica uma busca local final na melhor solução
    Individual bestSolution = population[0];
    for (int i = 0; i < 10; i++) { // Tenta algumas iterações de melhoria local
        for (size_t j = 0; j < bestSolution.routes.size(); j++) {
            // Tenta melhorar cada rota individualmente
            Route& route = bestSolution.routes[j];

            // Aplica 2-opt na rota (troca de segmentos)
            for (size_t k = 1; k < route.nodes.size() - 2; k++) {
                for (size_t l = k + 1; l < route.nodes.size() - 1; l++) {
                    double currentDist = calculateRouteSegmentDistance(route.nodes[k-1], route.nodes[k]) +
                                         calculateRouteSegmentDistance(route.nodes[l], route.nodes[l+1]);

                    double newDist = calculateRouteSegmentDistance(route.nodes[k-1], route.nodes[l]) +
                                     calculateRouteSegmentDistance(route.nodes[k], route.nodes[l+1]);

                    if (newDist < currentDist) {
                        // Inverte o segmento [k, l]
                        reverse(route.nodes.begin() + k, route.nodes.begin() + l + 1);
                    }
                }
            }
        }

        updateRouteDistances(bestSolution);
        bestSolution.calculateTotalDistance();
        bestSolution.calculateFitness();
    }

    // Atualiza a melhor solução se a busca local melhorou
    if (bestSolution.totalDistance < population[0].totalDistance) {
        cout << "Busca local final melhorou a solução:" << endl;
        cout << "Nova distância total: " << bestSolution.totalDistance
             << " (melhoria de " << population[0].totalDistance - bestSolution.totalDistance << ")" << endl;

        return bestSolution.routes;
    }

    return population[0].routes;
}

void GeneticSolver::printPopulationStats() const {
    cout << "\nEstatísticas da População:" << endl;
    cout << "Tamanho: " << population.size() << endl;

    if (population.empty()) {
        cout << "População vazia." << endl;
        return;
    }

    double totalFitness = 0;
    double bestFitness = population[0].fitness;
    double worstFitness = population.back().fitness;

    for (const auto& individual : population) {
        totalFitness += individual.fitness;
    }

    double avgFitness = totalFitness / population.size();

    cout << "Melhor fitness: " << bestFitness << " (distância: " << population[0].totalDistance << ")" << endl;
    cout << "Fitness médio: " << avgFitness << endl;
    cout << "Pior fitness: " << worstFitness << endl;

    // Diversidade (quantos indivíduos distintos)
    set<double> distinctFitness;
    for (const auto& individual : population) {
        distinctFitness.insert(individual.fitness);
    }

    cout << "Diversidade: " << distinctFitness.size() << " indivíduos distintos" << endl;
}

Individual GeneticSolver::getBestIndividual() const {
    if (population.empty()) {
        throw runtime_error("População vazia ao obter melhor indivíduo");
    }
    return population[0];
}

// Função para imprimir estatísticas detalhadas da evolução
void GeneticSolver::printEvolutionStats(int generation, double bestDistance, double avgDistance, double worstDistance) const {
    cout << "Geração " << setw(4) << generation << ": ";
    cout << "Melhor = " << setw(8) << fixed << setprecision(2) << bestDistance << " | ";
    cout << "Média = " << setw(8) << fixed << setprecision(2) << avgDistance << " | ";
    cout << "Pior = " << setw(8) << fixed << setprecision(2) << worstDistance << " | ";
    cout << "Rotas = " << setw(2) << population[0].routes.size() << endl;
}

void GeneticSolver::routeMergeMutation(Individual& individual) {
    if (individual.routes.size() < 2) return;  // Precisa de pelo menos 2 rotas

    // Identifica pares de rotas que poderiam ser mescladas
    vector<pair<size_t, size_t>> mergeCandidates;

    for (size_t i = 0; i < individual.routes.size(); i++) {
        for (size_t j = i+1; j < individual.routes.size(); j++) {
            Route& route1 = individual.routes[i];
            Route& route2 = individual.routes[j];

            // Verifica se a fusão não excede a capacidade
            if (route1.totalDemand + route2.totalDemand <= vehicleCapacity) {
                // Calcula o possível ganho de distância com a fusão
                // Suponha que a rota mesclada preserve a ordem dos nós de ambas as rotas

                // Conexão atual: depósito final da rota1 -> depósito inicial da rota2
                double currentConnection =
                        calculateRouteSegmentDistance(route1.nodes.back(), route2.nodes.front());

                // Possível nova conexão: último cliente da rota1 -> primeiro cliente da rota2
                double newConnection =
                        calculateRouteSegmentDistance(route1.nodes[route1.nodes.size()-2],
                                                      route2.nodes[1]);

                double distanceGain = currentConnection - newConnection;

                // Armazena o par de rotas e o ganho esperado
                mergeCandidates.push_back({i, j});
            }
        }
    }

    // Se não há candidatos viáveis, não faz nada
    if (mergeCandidates.empty()) return;

    // Seleciona um par aleatório dentre os candidatos
    uniform_int_distribution<size_t> candidateDist(0, mergeCandidates.size() - 1);
    auto selectedPair = mergeCandidates[candidateDist(rng)];

    size_t route1Idx = selectedPair.first;
    size_t route2Idx = selectedPair.second;

    Route& route1 = individual.routes[route1Idx];
    Route& route2 = individual.routes[route2Idx];

    // Mescla as duas rotas: insere os nós da rota2 (exceto depósito) na rota1 antes do depósito final
    route1.nodes.pop_back();  // Remove depósito final temporariamente

    // Adiciona os nós da rota2 (exceto depósito inicial)
    for (size_t i = 1; i < route2.nodes.size(); i++) {
        route1.nodes.push_back(route2.nodes[i]);
    }

    // Atualiza a demanda da rota mesclada
    route1.totalDemand += route2.totalDemand;

    // Remove a rota2 do indivíduo
    individual.routes.erase(individual.routes.begin() + route2Idx);
}