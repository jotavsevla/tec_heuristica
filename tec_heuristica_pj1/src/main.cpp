#include "../include/PhysarumSolver.h"
#include "../include/Interface.h"
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <filesystem>

using namespace std;

bool checkDirectoryStructure() {
    bool success = true;

    // Verifica se o diretório de entradas existe
    if (!filesystem::exists("entradas")) {
        cerr << "Erro: Diretório 'entradas' não encontrado!\n";
        success = false;
    }

    // Verifica se os arquivos XML existem
    for (int i = 1; i <= 14; i++) {
        string filename = string("entradas/CMT") + (i < 10 ? "0" : "") + to_string(i) + ".xml";
        if (!filesystem::exists(filename)) {
            cerr << "Aviso: Arquivo '" << filename << "' não encontrado!\n";
        }
    }

    // Cria diretório de resultados se não existir
    if (!filesystem::exists("resultados")) {
        try {
            filesystem::create_directory("resultados");
            cout << "Diretório 'resultados' criado com sucesso.\n";
        } catch (const exception& e) {
            cerr << "Erro ao criar diretório 'resultados': " << e.what() << "\n";
            success = false;
        }
    }

    return success;
}

void runWithInputFile() {
    cout << "Physarum CVRP Solver - Versão 1.0\n";
    cout << "=================================\n";

    // Verifica a estrutura de diretórios no início
    if (!checkDirectoryStructure()) {
        cout << "\nAtenção: Alguns problemas foram encontrados na estrutura de diretórios.\n";
        cout << "Deseja continuar mesmo assim? (s/n): ";
        char response;
        cin >> response;
        if (response != 's' && response != 'S') {
            return;
        }
    }
    string inputFile;
    cout << "Digite o nome do arquivo de entrada: ";
    cin >> inputFile;

    try {
        auto data = Interface::readInputFile(inputFile);
        PhysarumSolver solver(data.numNodes, data.numVehicles, data.vehicleCapacity);

        for (const auto& edge : data.edges) {
            solver.addEdge(edge.from, edge.to, edge.weight);
        }

        for (const auto& demand : data.demands) {
            solver.setDemand(demand.first, demand.second);
        }

        auto routes = solver.findRoutes();
        Interface::displayResults(routes);

        string outputFile = inputFile + ".out";
        Interface::writeOutputFile(outputFile, routes);
        cout << "\nResultados salvos em: " << outputFile << endl;

    } catch (const exception& e) {
        cerr << "Erro: " << e.what() << endl;
    }
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
    cout << "Physarum CVRP Solver - Versão 1.0\n";
    cout << "=================================\n";

    while (true) {
        try {
            Interface::showMenu();

            int choice;
            if (!(cin >> choice)) {
                cin.clear();
                cin.ignore(numeric_limits<streamsize>::max(), '\n');
                throw runtime_error("Entrada inválida. Por favor, digite um número.");
            }

            switch (choice) {
                case 1:
                    cout << "Funcionalidade de criar arquivo de exemplo não implementada.\n";
                    break;
                case 2:
                    runWithInputFile();
                    break;
                case 3:
                    runWithChristofidesFile();
                    break;
                case 4:
                    Interface::runAutomaticTest();
                    break;
                case 5:
                    cout << "Encerrando programa...\n";
                    return 0;
                default:
                    cout << "Opção inválida! Por favor, escolha uma opção válida.\n";
            }

        } catch (const exception& e) {
            cerr << "Erro: " << e.what() << endl;
            cout << "Pressione Enter para continuar...";
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cin.get();
        }

        cout << "\nPressione Enter para continuar...";
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cin.get();
    }

    return 0;
}