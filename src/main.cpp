#include "../include/Interface.h"
#include "../include/PhysarumSolver.h"
#include "../include/PhysarumSolverV3.h"
#include "../include/GeneticSolver.h"
#include <iostream>
#include <iomanip>
#include <chrono>

using namespace std;
using namespace std::chrono;

void displayComparativeResults(const string& instanceName,
                               const vector<Route>& physarumRoutes,
                               const vector<Route>& geneticRoutes) {
    cout << "\n==========================================\n";
    cout << "Resultados Comparativos para " << instanceName << "\n";
    cout << "==========================================\n\n";

    // Calcula custos totais
    double physarumCost = 0;
    double geneticCost = 0;

    for (const auto& route : physarumRoutes) {
        physarumCost += route.totalDistance;
    }
    for (const auto& route : geneticRoutes) {
        geneticCost += route.totalDistance;
    }

    // Exibe resultados do Physarum original
    cout << "PhysarumSolver Original:\n";
    cout << "------------------------\n";
    cout << "Número de rotas: " << physarumRoutes.size() << "\n";
    cout << "Distância total: " << physarumCost << "\n\n";

    // Exibe resultados do GeneticSolver
    cout << "GeneticSolver:\n";
    cout << "-------------\n";
    cout << "Número de rotas: " << geneticRoutes.size() << "\n";
    cout << "Distância total: " << geneticCost << "\n";

    // Calcula e exibe a melhoria
    double improvement = ((physarumCost - geneticCost) / physarumCost) * 100;
    cout << "\nMelhoria obtida: " << fixed << setprecision(2) << improvement << "%\n";
}

void runWithGeneticSolver() {
    cout << "Arquivos disponíveis:\n";
    cout << "=====================\n";
    for (int i = 1; i <= 14; i++) {
        cout << i << ". CMT" << (i < 10 ? "0" : "") << i << ".xml\n";
    }

    int choice;
    cout << "\nEscolha o número do arquivo (1-14): ";
    cin >> choice;

    if (choice < 1 || choice > 14) {
        throw runtime_error("Número de arquivo inválido");
    }

    // Constrói o nome do arquivo
    string filename = string("CMT") + (choice < 10 ? "0" : "") + to_string(choice) + ".xml";
    string fullPath = "entradas/" + filename;

    try {
        cout << "\nUsando arquivo: " << fullPath << endl;

        // Lê os dados da instância
        auto xmlData = Interface::readChristofidesXML(fullPath);
        cout << "XML lido com sucesso!" << endl;

        auto data = Interface::convertXMLToProblemData(xmlData);

        // ====== Executa PhysarumSolver Original ======
        cout << "\nExecutando PhysarumSolver original...\n";
        auto start = high_resolution_clock::now();

        PhysarumSolver solver(data.numNodes, data.numVehicles, data.vehicleCapacity, xmlData.depot);
        for (const auto& edge : data.edges) {
            solver.addEdge(edge.from, edge.to, edge.weight);
        }
        for (const auto& demand : data.demands) {
            solver.setDemand(demand.first, demand.second);
        }
        auto physarumRoutes = solver.findRoutes();

        auto physarumDuration = duration_cast<seconds>(high_resolution_clock::now() - start);
        cout << "Tempo de execução: " << physarumDuration.count() << " segundos\n";

        // Valida solução do Physarum original
        cout << "\nValidando solução do PhysarumSolver original...\n";
        Interface::validateChristofidesResult(fullPath, physarumRoutes);

        // ====== Executa GeneticSolver ======
        cout << "\nExecutando GeneticSolver com base no PhysarumSolver...\n";
        start = high_resolution_clock::now();

        // Define parâmetros do algoritmo genético
        int popSize = 50;
        int maxGen = 200;
        double crossRate = 0.85;
        double mutRate = 0.2;
        double eliteRate = 0.1;

        GeneticSolver geneticSolver(
                data.numNodes,
                data.numVehicles,
                data.vehicleCapacity,
                xmlData.depot,
                popSize,
                maxGen,
                crossRate,
                mutRate,
                eliteRate
        );

        for (const auto& edge : data.edges) {
            geneticSolver.addEdge(edge.from, edge.to, edge.weight);
        }
        for (const auto& demand : data.demands) {
            geneticSolver.setDemand(demand.first, demand.second);
        }
        auto geneticRoutes = geneticSolver.findRoutes();

        auto geneticDuration = duration_cast<seconds>(high_resolution_clock::now() - start);
        cout << "Tempo de execução: " << geneticDuration.count() << " segundos\n";

        // Valida solução do GeneticSolver
        cout << "\nValidando solução do GeneticSolver...\n";
        Interface::validateChristofidesResult(fullPath, geneticRoutes);

        // Exibe resultados comparativos
        displayComparativeResults(filename, physarumRoutes, geneticRoutes);

        // Salva resultados em arquivos
        string physarumOutFile = "resultados/" + filename + "_physarum.out";
        string geneticOutFile = "resultados/" + filename + "_genetic.out";

        Interface::writeOutputFile(physarumOutFile, physarumRoutes);
        Interface::writeOutputFile(geneticOutFile, geneticRoutes);

        cout << "\nResultados salvos em:\n";
        cout << "PhysarumSolver: " << physarumOutFile << endl;
        cout << "GeneticSolver: " << geneticOutFile << endl;

    } catch (const exception& e) {
        cerr << "Erro: " << e.what() << endl;
    }
}

int main() {
    try {
        cout << "\n=================================\n";
        cout << "    Genetic CVRP Solver Demo    \n";
        cout << "=================================\n";

        runWithGeneticSolver();

    } catch (const exception& e) {
        cerr << "Erro fatal: " << e.what() << endl;
        return 1;
    }

    return 0;
}