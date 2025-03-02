#ifndef EDGE_H
#define EDGE_H

struct Edge {
    int from;
    int to;
    double weight;
    Edge(int f, int t, double w);
};

#endif // EDGE_H