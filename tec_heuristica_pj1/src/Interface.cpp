#include "../include/Interface.h"
#include "../include/PhysarumSolver.h"
#include "../lab/tinyxml2/tinyxml2.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iomanip>
#include <cmath>

using namespace std;
using namespace tinyxml2;

XMLProblemData Interface::readChristofidesXML(const string& filename) {
    XMLProblemData data;
    XMLDocument doc;

    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS)
        throw runtime_error("Erro ao abrir arquivo XML");


    auto instance = doc.FirstChildElement("instance");
    auto network = instance->FirstChildElement("network");
    auto nodes = network->FirstChildElement("nodes");
    auto fleet = instance->FirstChildElement("fleet");
    auto requests = instance->FirstChildElement("requests");

    // Lê capacidade do veículo
    data.vehicleCapacity = fleet->FirstChildElement("vehicle_profile")
            ->FirstChildElement("capacity")
            ->DoubleText();

    // Lê nós e encontra o depósito
    for (auto node = nodes->FirstChildElement("node"); node;
         node = node->NextSiblingElement("node")) {
        int id = node->IntAttribute("id");
        int type = node->IntAttribute("type");
        double x = node->FirstChildElement("cx")->DoubleText();
        double y = node->FirstChildElement("cy")->DoubleText();

        data.nodes.push_back(Point(x, y));

        if (type == 0) { // Depósito
            data.depot = id - 1; // Converte para base-0
        }
    }

    // Lê demandas
    for (auto request = requests->FirstChildElement("request"); request;
         request = request->NextSiblingElement("request")) {
        int node = request->IntAttribute("node");
        double quantity = request->FirstChildElement("quantity")->DoubleText();
        data.demands[node - 1] = quantity; // Converte para base-0
    }

    // Calcula número de veículos necessários
    double totalDemand = 0;
    for (const auto& demand : data.demands) {
        totalDemand += demand.second;
    }
    data.numVehicles = ceil(totalDemand / data.vehicleCapacity);

    return data;
}

Interface::ProblemData Interface::convertXMLToProblemData(const XMLProblemData& xmlData) {
    ProblemData data;
    data.numNodes = xmlData.nodes.size();
    data.numVehicles = xmlData.numVehicles;
    data.vehicleCapacity = xmlData.vehicleCapacity;
    data.demands = xmlData.demands;

    // Adiciona demanda zero para o depósito
    data.demands[xmlData.depot] = 0;

    // Calcula distâncias euclidianas entre todos os nós
    for (int i = 0; i < data.numNodes; i++) {
        for (int j = i + 1; j < data.numNodes; j++) {
            double dx = xmlData.nodes[i].x - xmlData.nodes[j].x;
            double dy = xmlData.nodes[i].y - xmlData.nodes[j].y;
            double distance = round(sqrt(dx * dx + dy * dy));
            data.edges.push_back(Edge(i, j, distance));
        }
    }

    return data;
}

Interface::ProblemData Interface::readInputFile(const string& filename) {
    cout << "Tentando abrir arquivo: " << filename << endl;

    ifstream file(filename);
    if (!file.is_open()) {
        throw runtime_error("Não foi possível abrir o arquivo: " + filename);
    }

    ProblemData data;
    string line;
    int lineNumber = 0;

    try {
        getline(file, line);
        lineNumber++;

        cout << "Lendo configuração inicial..." << endl;
        istringstream iss(line);
        if (!(iss >> data.numNodes >> data.numVehicles >> data.vehicleCapacity)) {
            throw runtime_error("Erro ao ler configuração inicial na linha " +
                                to_string(lineNumber));
        }

        // Validações básicas
        if (data.numNodes <= 0 || data.numVehicles <= 0 || data.vehicleCapacity <= 0) {
            throw runtime_error("Valores inválidos na configuração inicial");
        }

        cout << "Configuração lida:\n"
             << "Nós: " << data.numNodes << "\n"
             << "Veículos: " << data.numVehicles << "\n"
             << "Capacidade: " << data.vehicleCapacity << endl;

        // Lê arestas
        bool foundEdges = false;
        while (getline(file, line)) {
            lineNumber++;
            if (line == "EDGES") {
                foundEdges = true;
                break;
            }
        }

        if (!foundEdges) {
            throw runtime_error("Seção EDGES não encontrada");
        }

        while (getline(file, line) && line != "END_EDGES") {
            lineNumber++;
            if (line.empty() || line[0] == '#') continue;

            int from, to;
            double weight;
            istringstream edge_iss(line);
            if (!(edge_iss >> from >> to >> weight)) {
                throw runtime_error("Erro ao ler aresta na linha " +
                                    to_string(lineNumber));
            }
            data.edges.push_back(Edge(from, to, weight));
        }

        // Lê demandas
        bool foundDemands = false;
        while (getline(file, line)) {
            lineNumber++;
            if (line == "DEMANDS") {
                foundDemands = true;
                break;
            }
        }

        if (!foundDemands) {
            throw runtime_error("Seção DEMANDS não encontrada");
        }

        while (getline(file, line) && line != "END_DEMANDS") {
            lineNumber++;
            if (line.empty() || line[0] == '#') continue;

            int node;
            double demand;
            istringstream demand_iss(line);
            if (!(demand_iss >> node >> demand)) {
                throw runtime_error("Erro ao ler demanda na linha " +
                                    to_string(lineNumber));
            }
            data.demands[node] = demand;
        }

        // Validações finais
        if (data.edges.empty() || data.demands.empty()) {
            throw runtime_error("Dados incompletos no arquivo");
        }

    } catch (const exception& e) {
        throw runtime_error(string("Erro ao ler arquivo na linha ") +
                            to_string(lineNumber) + ": " + e.what());
    }

    file.close();
    return data;
}

void Interface::displayResults(const vector<Route>& routes) {
    cout << "\nResultados encontrados:\n";
    cout << "=====================\n";

    double totalDistance = 0;
    for (size_t i = 0; i < routes.size(); ++i) {
        cout << "\nRota " << (i + 1) << ":\n";
        cout << "Sequência: ";
        for (size_t j = 0; j < routes[i].nodes.size(); ++j) {
            cout << routes[i].nodes[j];
            if (j < routes[i].nodes.size() - 1) cout << " -> ";
        }
        cout << "\nDemanda total: " << routes[i].totalDemand;
        cout << "\nDistância: " << routes[i].totalDistance << "\n";
        totalDistance += routes[i].totalDistance;
    }

    cout << "\nDistância total: " << totalDistance << "\n";
}

void Interface::writeOutputFile(const string& filename, const vector<Route>& routes) {
    ofstream file(filename);
    if (!file.is_open()) {
        throw runtime_error("Não foi possível criar o arquivo de saída");
    }

    file << "Solução CVRP\n";
    file << "============\n\n";

    double totalDistance = 0;
    for (size_t i = 0; i < routes.size(); ++i) {
        file << "Rota " << (i + 1) << ":\n";
        file << "Sequência: ";
        for (size_t j = 0; j < routes[i].nodes.size(); ++j) {
            file << routes[i].nodes[j];
            if (j < routes[i].nodes.size() - 1) file << " -> ";
        }
        file << "\nDemanda total: " << routes[i].totalDemand;
        file << "\nDistância: " << routes[i].totalDistance << "\n\n";
        totalDistance += routes[i].totalDistance;
    }

    file << "Distância total: " << totalDistance << "\n";
    file.close();
}

void Interface::showMenu() {
    cout << "\n=================================\n";
    cout << "    Physarum CVRP Solver Menu    \n";
    cout << "=================================\n";
    cout << "1. Criar arquivo de exemplo (8 nós)\n";
    cout << "2. Resolver problema de arquivo texto\n";
    cout << "3. Resolver problema de arquivo XML (Christofides)\n";
    cout << "4. Executar teste automático\n";
    cout << "5. Sair\n";
    cout << "=================================\n";
    cout << "Escolha uma opção: ";
}

void Interface::runAutomaticTest() {
    int numNodes = 6;
    int numVehicles = 2;
    double vehicleCapacity = 30.0;
    int depot = 0;

    PhysarumSolver solver(numNodes, numVehicles, vehicleCapacity, depot);

    solver.addEdge(0, 1, 1.0);
    solver.addEdge(0, 2, 2.0);
    solver.addEdge(1, 2, 1.0);
    solver.addEdge(1, 3, 2.0);
    solver.addEdge(2, 3, 1.0);
    solver.addEdge(2, 4, 2.0);
    solver.addEdge(3, 4, 1.0);
    solver.addEdge(3, 5, 2.0);
    solver.addEdge(4, 5, 1.0);

    solver.setDemand(0, 0.0);
    solver.setDemand(1, 10.0);
    solver.setDemand(2, 15.0);
    solver.setDemand(3, 12.0);
    solver.setDemand(4, 8.0);
    solver.setDemand(5, 14.0);

    cout << "\nExecutando teste automático com valores padrão:\n";
    cout << "Número de nós: " << numNodes << "\n";
    cout << "Número de veículos: " << numVehicles << "\n";
    cout << "Capacidade dos veículos: " << vehicleCapacity << "\n";

    auto routes = solver.findRoutes();
    displayResults(routes);
}