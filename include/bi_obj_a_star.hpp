#pragma once
#include "landmarks.hpp"
#include "graph_parse.hpp"
#include <cstdint>
#include <vector>

struct Node {
    uint32_t state;
    uint32_t g1, g2;  
    uint32_t f1, f2;

    bool operator>(const Node& other) const;
};

struct Result {
    std::vector<Node> solutions;
    std::vector<uint32_t> lower_bound; 
};

class BOBALT {
public:
    BOBALT(const Graph &g, const ReverseGraph &rg, const std::vector<LandmarkTable> &lm);
    std::vector<Node> query(uint32_t source, uint32_t target);

private:
    const Graph &graph;
    const ReverseGraph &rev_graph;
    const std::vector<LandmarkTable> &landmarks;

    uint32_t alt_h(int metric, uint32_t u, uint32_t v) const;
};
