#ifndef SOLUTION_VALIDATOR_H
#define SOLUTION_VALIDATOR_H

#include <vector>
#include <map>
#include <set>
#include <string>
#include "../lab/tinyxml2/tinyxml2.h"
#include "Types.h"  // Para struct Point
#include "Route.h"  // Para struct Route

class SolutionValidator {
private:
    std::vector<Point> nodes;
    std::map<int, double> demands;
    double vehicleCapacity;
    int depot;
    std::vector<Route> solution;
    std::set<int> visitedNodes;
    
    double calculateDistance(const Point& p1, const Point& p2);

public:
    void loadXMLData(const std::string& filename);
    void loadSolution(const std::vector<Route>& routes);
    bool validateSolution();
};

#endif // SOLUTION_VALIDATOR_H