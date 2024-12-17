#ifndef INTERFACE_H
#define INTERFACE_H

#include "Types.h"
#include "Edge.h"
#include "Route.h"
#include "PhysarumSolver.h"
#include <string>
#include <map>
#include <vector>

using namespace std;

class Interface {
public:
    struct ProblemData {
        int numNodes;
        int numVehicles;
        double vehicleCapacity;
        vector<Edge> edges;
        map<int, double> demands;
    };

    static ProblemData readInputFile(const string& filename);
    static XMLProblemData readChristofidesXML(const string& filename);
    static ProblemData convertXMLToProblemData(const XMLProblemData& xmlData);
    static void displayResults(const vector<Route>& routes);
    static void writeOutputFile(const string& filename, const vector<Route>& routes);
    static void showMenu();
    static void runAutomaticTest();
    static void validateChristofidesResult(const string& xmlFile, const vector<Route>& routes);
};

#endif // INTERFACE_H