#pragma once
#include "graph_parse.hpp"
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

static constexpr uint32_t INF = std::numeric_limits<uint32_t>::max() / 2;

enum class LandmarkPolicy {
    Random,
    Farthest,
    OptimizedFarthest,
    Avoid,
};

struct LandmarkTable {
    uint32_t num_landmarks = 0;
    uint32_t num_nodes     = 0;
    std::vector<uint32_t> nodes;
    std::vector<uint32_t>  dist_from; // dist_from[lm * num_nodes + v] = dist(lm, v)
    std::vector<uint32_t>  dist_to;   // dist_to [lm * num_nodes + v] = dist(v, lm)

    uint32_t from(uint32_t lm, uint32_t v) const { return dist_from[lm * num_nodes + v]; }
    uint32_t to  (uint32_t lm, uint32_t v) const { return dist_to  [lm * num_nodes + v]; }
    uint32_t lower_bound(uint32_t u, uint32_t v) const;
};

template <typename GraphType>
void dijkstra(const GraphType &g, uint32_t source, int metric,
              uint32_t *dists, uint32_t *parents = nullptr);

LandmarkTable build_landmarks(const Graph &g, const ReverseGraph &rg,
                              uint32_t num_lm, int metric,
                              LandmarkPolicy policy,
                              uint32_t seed = 0);

void save_landmarks(const LandmarkTable &t, const std::string &path);
LandmarkTable load_landmarks(const std::string &path);