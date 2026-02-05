#include "../include/Interface.h"
#include "../include/PhysarumSolverV3.h"
#include <iostream>
#include <iomanip>
#include <chrono>

using namespace std;
using namespace std::chrono;

void displayComparativeResults(const string& instanceName,
                               const vector<Route>& physarumRoutes,
                               const vector<Route>& physarumV3Routes) {
    cout << "\n==========================================\n";
    cout << "Resultados Comparativos para " << instanceName << "\n";
    cout << "==========================================\n\n";

    // Calcula custos totais
    double physarumCost = 0;
    double physarumV3Cost = 0;

    for (const auto& route : physarumRoutes) {
        physarumCost += route.totalDistance;
    }
    for (const auto& route : physarumV3Routes) {
        physarumV3Cost += route.totalDistance;
    }

    // Exibe resultados do Physarum original
    cout << "PhysarumSolver Original:\n";
    cout << "------------------------\n";
    cout << "Número de rotas: " << physarumRoutes.size() << "\n";
    cout << "Distância total: " << physarumCost << "\n\n";

    // Exibe resultados do PhysarumV3
    cout << "PhysarumSolverV3 (com busca local e tabu):\n";
    cout << "----------------------------------------\n";
    cout << "Número de rotas: " << physarumV3Routes.size() << "\n";
    cout << "Distância total: " << physarumV3Cost << "\n";

    // Calcula e exibe a melhoria
    double improvement = ((physarumCost - physarumV3Cost) / physarumCost) * 100;
    cout << "\nMelhoria obtida: " << fixed << setprecision(2) << improvement << "%\n";
}

void runWithChristofidesFile() {
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
    string fullPath = "/Users/jotavsevla/CLionProjects/tec_heuristica/tec_heuristica_pj1/entradas/" + filename;

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

        // ====== Executa PhysarumSolverV3 ======
        cout << "\nExecutando PhysarumSolverV3 com busca local e tabu...\n";
        start = high_resolution_clock::now();

        PhysarumSolverV3 solverV3(data.numNodes, data.numVehicles, data.vehicleCapacity, xmlData.depot);
        for (const auto& edge : data.edges) {
            solverV3.addEdge(edge.from, edge.to, edge.weight);
        }
        for (const auto& demand : data.demands) {
            solverV3.setDemand(demand.first, demand.second);
        }
        auto physarumV3Routes = solverV3.findRoutes();

        auto physarumV3Duration = duration_cast<seconds>(high_resolution_clock::now() - start);
        cout << "Tempo de execução: " << physarumV3Duration.count() << " segundos\n";

        // Valida solução do PhysarumV3
        cout << "\nValidando solução do PhysarumSolverV3...\n";
        Interface::validateChristofidesResult(fullPath, physarumV3Routes);

        // Exibe resultados comparativos
        displayComparativeResults(filename, physarumRoutes, physarumV3Routes);

        // Salva resultados em arquivos
        string physarumOutFile = "resultados/" + filename + "_physarum.out";
        string physarumV3OutFile = "resultados/" + filename + "_physarumV3.out";

        Interface::writeOutputFile(physarumOutFile, physarumRoutes);
        Interface::writeOutputFile(physarumV3OutFile, physarumV3Routes);

        cout << "\nResultados salvos em:\n";
        cout << "PhysarumSolver: " << physarumOutFile << endl;
        cout << "PhysarumSolverV3: " << physarumV3OutFile << endl;

    } catch (const exception& e) {
        cerr << "Erro: " << e.what() << endl;
    }
}

int main() {
    try {
        while (true) {
            Interface::showMenu();
            int choice;
            cin >> choice;
            cin.ignore(numeric_limits<streamsize>::max(), '\n');

            switch (choice) {
                case 1: {
                    cout << "Funcionalidade não implementada.\n";
                    break;
                }
                case 2: {
                    cout << "Digite o nome do arquivo de entrada: ";
                    string filename;
                    getline(cin, filename);

                    try {
                        auto data = Interface::readInputFile(filename);

                        // Executa PhysarumSolver original
                        cout << "\nExecutando PhysarumSolver original...\n";
                        PhysarumSolver solver(data.numNodes, data.numVehicles,
                                              data.vehicleCapacity, 0);
                        for (const auto& edge : data.edges) {
                            solver.addEdge(edge.from, edge.to, edge.weight);
                        }
                        for (const auto& demand : data.demands) {
                            solver.setDemand(demand.first, demand.second);
                        }
                        auto physarumRoutes = solver.findRoutes();

                        // Executa PhysarumSolverV3
                        cout << "\nExecutando PhysarumSolverV3...\n";
                        PhysarumSolverV3 solverV3(data.numNodes, data.numVehicles,
                                                  data.vehicleCapacity, 0);
                        for (const auto& edge : data.edges) {
                            solverV3.addEdge(edge.from, edge.to, edge.weight);
                        }
                        for (const auto& demand : data.demands) {
                            solverV3.setDemand(demand.first, demand.second);
                        }
                        auto physarumV3Routes = solverV3.findRoutes();

                        // Exibe resultados comparativos
                        displayComparativeResults(filename, physarumRoutes, physarumV3Routes);

                        cout << "\nDeseja salvar os resultados? (s/n): ";
                        string save;
                        getline(cin, save);
                        if (save == "s" || save == "S") {
                            cout << "Digite o nome base do arquivo de saída: ";
                            string outfileBase;
                            getline(cin, outfileBase);

                            string physarumOutFile = outfileBase + "_physarum.out";
                            string physarumV3OutFile = outfileBase + "_physarumV3.out";

                            Interface::writeOutputFile(physarumOutFile, physarumRoutes);
                            Interface::writeOutputFile(physarumV3OutFile, physarumV3Routes);

                            cout << "Resultados salvos em: " << physarumOutFile << " e " << physarumV3OutFile << endl;
                        }
                    } catch (const exception& e) {
                        cerr << "Erro: " << e.what() << endl;
                    }
                    break;
                }
                case 3: runWithChristofidesFile();
                    break;
                case 4:
                    cout << "Encerrando programa...\n";
                    return 0;
                default:
                    cout << "Opção inválida!\n";
            }
        }
    } catch (const exception& e) {
        cerr << "Erro fatal: " << e.what() << endl;
        return 1;
    }

    return 0;
}