#include "../include/SolutionValidator.h"
#include "../lab/tinyxml2/tinyxml2.h"
#include <iostream>
#include <cmath>

using namespace std;
using namespace tinyxml2;

double SolutionValidator::calculateDistance(const Point& p1, const Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return round(sqrt(dx * dx + dy * dy));
}

void SolutionValidator::loadXMLData(const string& filename) {
    XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS)
        throw runtime_error("Erro ao abrir arquivo XML");

    auto instance = doc.FirstChildElement("instance");
    auto network = instance->FirstChildElement("network");
    auto nodes_xml = network->FirstChildElement("nodes");
    auto fleet = instance->FirstChildElement("fleet");
    auto requests = instance->FirstChildElement("requests");

    // Lê capacidade do veículo
    vehicleCapacity = fleet->FirstChildElement("vehicle_profile")
            ->FirstChildElement("capacity")
            ->DoubleText();

    // Lê nós
    for (auto node = nodes_xml->FirstChildElement("node"); node;
         node = node->NextSiblingElement("node")) {
        int id = node->IntAttribute("id");
        int type = node->IntAttribute("type");
        double x = node->FirstChildElement("cx")->DoubleText();
        double y = node->FirstChildElement("cy")->DoubleText();

        // Expande o vetor de nós se necessário
        if (id > static_cast<int>(nodes.size())) {
            nodes.resize(id);
        }
        nodes[id-1] = Point(x, y);

        if (type == 0) { // Depósito
            depot = id - 1;
        }
    }

    // Lê demandas
    for (auto request = requests->FirstChildElement("request"); request;
         request = request->NextSiblingElement("request")) {
        int node = request->IntAttribute("node");
        double quantity = request->FirstChildElement("quantity")->DoubleText();
        demands[node - 1] = quantity;
    }
}

void SolutionValidator::loadSolution(const vector<Route>& routes) {
    solution = routes;
}

bool SolutionValidator::validateSolution() {
    bool isValid = true;
    visitedNodes.clear();
    double totalDistance = 0;

    cout << "\nIniciando validação da solução...\n";
    cout << "===================================\n";

    // Para cada rota
    for (size_t i = 0; i < solution.size(); i++) {
        cout << "\nValidando Rota " << (i + 1) << ":\n";
        
        double routeDemand = 0;
        double routeDistance = 0;
        
        // Verifica se começa e termina no depósito
        if (solution[i].nodes.front() != depot || solution[i].nodes.back() != depot) {
            cout << "ERRO: Rota " << (i + 1) << " não começa/termina no depósito\n";
            isValid = false;
        }

        // Calcula demanda e distância da rota
        for (size_t j = 0; j < solution[i].nodes.size(); j++) {
            int currentNode = solution[i].nodes[j];
            
            // Verifica duplicatas (exceto depósito)
            if (currentNode != depot) {
                if (visitedNodes.count(currentNode) > 0) {
                    cout << "ERRO: Nó " << currentNode << " visitado múltiplas vezes\n";
                    isValid = false;
                }
                visitedNodes.insert(currentNode);
                routeDemand += demands[currentNode];
            }

            // Calcula distância
            if (j > 0) {
                int prevNode = solution[i].nodes[j-1];
                double distance = calculateDistance(nodes[prevNode], nodes[currentNode]);
                routeDistance += distance;
            }
        }

        // Verifica capacidade
        if (routeDemand > vehicleCapacity) {
            cout << "ERRO: Capacidade excedida na rota " << (i + 1) 
                 << " (Demanda: " << routeDemand << ", Capacidade: " 
                 << vehicleCapacity << ")\n";
            isValid = false;
        }

        // Verifica se a distância calculada corresponde à informada
        if (abs(routeDistance - solution[i].totalDistance) > 0.1) {
            cout << "ERRO: Distância incorreta na rota " << (i + 1) 
                 << " (Calculado: " << routeDistance 
                 << ", Informado: " << solution[i].totalDistance << ")\n";
            isValid = false;
        }

        // Verifica se a demanda calculada corresponde à informada
        if (abs(routeDemand - solution[i].totalDemand) > 0.1) {
            cout << "ERRO: Demanda incorreta na rota " << (i + 1)
                 << " (Calculado: " << routeDemand
                 << ", Informado: " << solution[i].totalDemand << ")\n";
            isValid = false;
        }

        totalDistance += routeDistance;
        
        cout << "Demanda calculada: " << routeDemand << "\n";
        cout << "Distância calculada: " << routeDistance << "\n";
    }

    // Verifica se todos os nós foram visitados
    for (const auto& demand : demands) {
        if (visitedNodes.count(demand.first) == 0) {
            cout << "ERRO: Nó " << demand.first << " não foi visitado\n";
            isValid = false;
        }
    }

    cout << "\nDistância total calculada: " << totalDistance << "\n";
    cout << "===================================\n";
    cout << "Solução é " << (isValid ? "VÁLIDA" : "INVÁLIDA") << "\n";

    return isValid;
}