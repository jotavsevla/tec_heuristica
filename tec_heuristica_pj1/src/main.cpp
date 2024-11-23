#include "../include/Interface.h"
#include "../include/PhysarumSolver.h"
#include <iostream>

using namespace std;
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

        auto xmlData = Interface::readChristofidesXML(fullPath);
        cout << "XML lido com sucesso!" << endl;

        auto data = Interface::convertXMLToProblemData(xmlData);
        PhysarumSolver solver(data.numNodes, data.numVehicles, data.vehicleCapacity, xmlData.depot);

        for (const auto& edge : data.edges) {
            solver.addEdge(edge.from, edge.to, edge.weight);
        }
        for (const auto& demand : data.demands) {
            solver.setDemand(demand.first, demand.second);
        }
        auto routes = solver.findRoutes();
        Interface::displayResults(routes);

        string outputFile = "resultados/" + filename + ".out";
        Interface::writeOutputFile(outputFile, routes);
        cout << "\nResultados salvos em: " << outputFile << endl;

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
                        PhysarumSolver solver(data.numNodes, data.numVehicles,
                                              data.vehicleCapacity, 0);  // Assume depósito em 0

                        // Adiciona arestas
                        for (const auto& edge : data.edges) {
                            solver.addEdge(edge.from, edge.to, edge.weight);
                        }

                        // Adiciona demandas
                        for (const auto& demand : data.demands) {
                            solver.setDemand(demand.first, demand.second);
                        }

                        auto routes = solver.findRoutes();
                        Interface::displayResults(routes);

                        cout << "\nDeseja salvar os resultados? (s/n): ";
                        string save;
                        getline(cin, save);
                        if (save == "s" || save == "S") {
                            cout << "Digite o nome do arquivo de saída: ";
                            string outfile;
                            getline(cin, outfile);
                            Interface::writeOutputFile(outfile, routes);
                        }
                    } catch (const exception& e) {
                        cerr << "Erro: " << e.what() << endl;
                    }
                    break;
                }
                case 3: runWithChristofidesFile();
                    break;
                case 4:
                    Interface::runAutomaticTest();
                    break;
                case 5:
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