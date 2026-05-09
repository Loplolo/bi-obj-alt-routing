#pragma once
#include <cstdint>
#include <string>
#include <vector>

struct Graph {
  struct Coordinate {
    int32_t x, y;
  };
  std::vector<uint32_t> offset;
  std::vector<uint32_t> target;
  std::vector<int32_t> distance;
  std::vector<int32_t> travel_time;
  std::vector<Coordinate> coords;
  size_t num_nodes = 0;
  size_t num_edges = 0;
};

Graph parse_gz(const std::string &dist_file, const std::string &time_file,
               const std::string &coord_file);

struct ReverseGraph : Graph {
  std::vector<uint32_t> offset;
  std::vector<uint32_t> source;
  std::vector<int32_t> distance;
  std::vector<int32_t> travel_time;

  explicit ReverseGraph(const Graph &g) {
    const size_t N = g.num_nodes;
    offset.assign(N + 1, 0);

    for (uint32_t u = 0; u < N; ++u)
      for (uint32_t e = g.offset[u]; e < g.offset[u + 1]; ++e)
        offset[g.target[e] + 1]++;

    for (size_t i = 1; i <= N; ++i)
      offset[i] += offset[i - 1];

    source.resize(g.num_edges);
    distance.resize(g.num_edges);
    travel_time.resize(g.num_edges);
    std::vector<uint32_t> pos = offset;

    for (uint32_t u = 0; u < N; ++u) {
      for (uint32_t e = g.offset[u]; e < g.offset[u + 1]; ++e) {
        uint32_t v = g.target[e];
        uint32_t idx = pos[v]++;
        source[idx] = u;
        distance[idx] = g.distance[e];
        travel_time[idx] = g.travel_time[e];
      }
    }
  }
};
