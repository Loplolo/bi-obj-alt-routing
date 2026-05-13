#pragma once
#include "landmarks.hpp"
#include "graph_parse.hpp"
#include <cstdint>
#include <vector>

struct Node {
    uint32_t state;
    int32_t g1, g2;
    int32_t f1, f2;
    int32_t parent;

    bool operator>(const Node& other) const;
};

class BOBALT {
public:
    BOBALT(const Graph &g, const std::vector<LandmarkTable> &lm);

    std::vector<Node> query(uint32_t source, uint32_t target);

private:
    const Graph &graph;
    const std::vector<LandmarkTable> &landmarks;

    int32_t h1(uint32_t u, uint32_t target) const;
    int32_t h2(uint32_t u, uint32_t target) const;
    int32_t ub1(uint32_t u, uint32_t target) const;
    int32_t ub2(uint32_t u, uint32_t target) const;
};
