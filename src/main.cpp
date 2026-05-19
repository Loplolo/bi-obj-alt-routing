#include "bi_obj_a_star.hpp"
#include "graph_parse.hpp"
#include "landmarks.hpp"
#include <filesystem>
#include <iostream>

static constexpr uint32_t NUM_LANDMARKS = 16;

int main(int argc, char *argv[]) {
    if (argc < 6) {
        std::cerr << "Usage: " << argv[0]
                  << " <dist.gr.gz> <time.gr.gz> <coord.co.gz>"
                     " <landmarks_dist.bin> <landmarks_time.bin>\n";
        return 1;
    }

    std::cout << "Parsing graph... " << std::flush;
    Graph g = parse_gz(argv[1], argv[2], argv[3]);
    std::cout << g.num_nodes << " nodes, " << g.num_edges << " edges\n";

    std::cout << "Building reverse graph... " << std::flush;
    ReverseGraph rg(g);
    std::cout << "done\n";

    std::vector<LandmarkTable> landmarks;

    for (int metric = 1; metric <= 2; ++metric) {
        const char *path = (metric == 1) ? argv[4] : argv[5];
        if (std::filesystem::exists(path)) {
            std::cout << "Loading landmarks from " << path << "... " << std::flush;
            landmarks.push_back(load_landmarks(path));
        } else {
            std::cout << "Building landmarks for" << (metric == 1 ? " distance" : " time") << "... " << std::flush;
            landmarks.push_back(
                build_landmarks(g, rg, NUM_LANDMARKS, metric, LandmarkPolicy::Farthest));
            save_landmarks(landmarks.back(), path);
        }
        std::cout << "done\n";
    }
    BOBALT algo(g, rg, landmarks);

    std::cout << "\n Enter source and target node IDs (enter 0 to quit).\n";
    while (true) {
        uint32_t src, tgt;
        std::cout << "source: ";
        if (!(std::cin >> src) || src == 0) break;
        std::cout << "target: ";
        if (!(std::cin >> tgt) || tgt == 0) break;

        if (src > g.num_nodes || tgt > g.num_nodes) {
            std::cout << "Out of range (1–" << g.num_nodes << ")\n";
            continue;
        }

        auto sols = algo.query(src - 1, tgt - 1);
        if (sols.empty()) {
            std::cout << "No path found.\n";
        } else {
            std::cout << sols.size() << " Pareto-optimal path(s):\n";
            for (const auto &n : sols)
                std::cout << "  distance=" << n.g1 << "  travel_time=" << n.g2 << "\n";
        }
    }
}
