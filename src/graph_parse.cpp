#include "graph_parse.hpp"
#include <zlib.h>
#include <sstream>
#include <cmath>
#include <cstdio>

uint32_t euclidean_dist(const Graph &graph, uint32_t u, uint32_t v) {
    const auto &cu = graph.coords[u];
    const auto &cv = graph.coords[v];
    const double dx = static_cast<double>(cu.x) - cv.x;
    const double dy = static_cast<double>(cu.y) - cv.y;
    return static_cast<uint32_t>(std::sqrt(dx * dx + dy * dy));
}

Graph parse_gz(const std::string &dist_file,
               const std::string &time_file,
               const std::string &coord_file) {
    Graph graph;
    char buffer[1024];
    std::string dummy;
    std::istringstream stream;

    gzFile coord_gz = gzopen(coord_file.c_str(), "rb");
    if (!coord_gz) fprintf(stderr, "Warning: cannot open coord file: %s\n", coord_file.c_str());
    if (coord_gz) {
        while (gzgets(coord_gz, buffer, sizeof(buffer))) {
            // p aux sp co num_nodes
            // p aux sp co 23947347   
            if (buffer[0] == 'p') {
                stream.clear(); stream.str(buffer + 2);
                uint32_t nodes;
                if (stream >> dummy >> dummy >> dummy >> nodes) {
                    graph.num_nodes = nodes;
                    graph.coords.resize(nodes);
                }
            } else if (buffer[0] == 'v') {
                // v id x y 
                // v 1 -86436719 32469271
                stream.clear();
                stream.str(buffer + 2);
                uint32_t id; 
                uint32_t x, y;
                if (stream >> id >> x >> y)
                    graph.coords[id - 1] = {x, y};
            }
        }
        // c comments lines
        gzclose(coord_gz);
    }

    struct Edge { uint32_t u, v; uint32_t w; };
    std::vector<Edge> dist_edges, time_edges;

    gzFile dist_gz = gzopen(dist_file.c_str(), "rb");
    if (!dist_gz) { fprintf(stderr, "Error: cannot open dist file: %s\n", dist_file.c_str()); return graph; }

    while (gzgets(dist_gz, buffer, sizeof(buffer))) {
        if (buffer[0] == 'p') {
            // p sp num_nodes num_edges
            // p sp 23947347 58333344
            stream.clear(); stream.str(buffer + 2);
            uint32_t nodes, edges;
            if (stream >> dummy >> nodes >> edges) {
                graph.num_nodes = nodes;
                graph.num_edges = edges;
                dist_edges.reserve(edges);
            }
        } else if (buffer[0] == 'a') {
            // a source target weight
            // a 1048577 1048578 414
            stream.clear(); stream.str(buffer + 2);
            uint32_t u, v; uint32_t w;
            if (stream >> u >> v >> w)
                dist_edges.push_back({u, v, w});
        }
    }
    gzclose(dist_gz);

    graph.offset.assign(graph.num_nodes + 1, 0);
    for (const auto &e : dist_edges)
        if (e.u >= 1 && e.u <= graph.num_nodes)
            graph.offset[e.u]++;
    for (size_t i = 1; i <= graph.num_nodes; ++i)
        graph.offset[i] += graph.offset[i - 1];

    graph.target.resize(graph.num_edges);
    graph.distance.resize(graph.num_edges);
    graph.travel_time.resize(graph.num_edges);

    gzFile time_gz = gzopen(time_file.c_str(), "rb");
    if (!time_gz) { fprintf(stderr, "Error: cannot open time file: %s\n", time_file.c_str()); return graph; }

    time_edges.reserve(graph.num_edges);
    while (gzgets(time_gz, buffer, sizeof(buffer))) {
        if (buffer[0] == 'a') {
            stream.clear(); stream.str(buffer + 2);
            uint32_t u, v; uint32_t w;
            if (stream >> u >> v >> w)
                time_edges.push_back({u, v, w});
        }
    }
    gzclose(time_gz);

    std::vector<uint32_t> current_pos = graph.offset;
    size_t n = std::min(dist_edges.size(), time_edges.size());
    for (size_t i = 0; i < n; ++i) {
        uint32_t u = dist_edges[i].u;
        if (u < 1 || u > graph.num_nodes) continue;
        uint32_t idx = current_pos[u - 1]++;
        if (idx >= graph.target.size()) continue;
        graph.target[idx] = dist_edges[i].v - 1;
        graph.distance[idx] = dist_edges[i].w;
        graph.travel_time[idx] = time_edges[i].w;
    }
    return graph;
}