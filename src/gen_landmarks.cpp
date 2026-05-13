#include "graph_parse.hpp"
#include "landmarks.hpp"
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

static void usage(const char *prog) {
    std::cerr <<
        "Usage: " << prog << " <dist.gr.gz> <time.gr.gz> <coord.co.gz> <output.bin>\n"
        "          [--metric <1|2>] [--num-lm <n>] [--policy <name>] [--seed <n>]\n"
        "\n"
        "Options:\n"
        "  --metric   1=distance  2=time  (default: 1)\n"
        "  --num-lm   number of landmarks (default: 16)\n"
        "  --policy   random | farthest | optimized | avoid | maxcover  (default: farthest)\n"
        "  --seed     RNG seed (default: 0)\n";
}

static LandmarkPolicy parse_policy(const std::string &s) {
    if (s == "random")    return LandmarkPolicy::Random;
    if (s == "farthest")  return LandmarkPolicy::Farthest;
    if (s == "optimized") return LandmarkPolicy::OptimizedFarthest;
    if (s == "avoid")     return LandmarkPolicy::Avoid;
    if (s == "maxcover")  return LandmarkPolicy::MaxCover;
    throw std::invalid_argument("Unknown policy: " + s);
}

int main(int argc, char *argv[]) {
    if (argc < 5) { usage(argv[0]); return 1; }

    const std::string dist_file  = argv[1];
    const std::string time_file  = argv[2];
    const std::string coord_file = argv[3];
    const std::string out_path   = argv[4];

    int      metric   = 1;
    uint32_t num_lm   = 16;
    uint32_t seed     = 0;
    LandmarkPolicy policy = LandmarkPolicy::Farthest;

    for (int i = 5; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--metric" || arg == "--num-lm" ||
             arg == "--policy" || arg == "--seed") && i + 1 >= argc) {
            std::cerr << "Missing value for " << arg << "\n";
            return 1;
        }
        if (arg == "--metric") {
            metric = std::stoi(argv[++i]);
            if (metric != 1 && metric != 2) {
                std::cerr << "--metric must be 1 or 2\n"; return 1;
            }
        } else if (arg == "--num-lm") {
            num_lm = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--policy") {
            try { policy = parse_policy(argv[++i]); }
            catch (const std::invalid_argument &e) {
                std::cerr << e.what() << "\n"; return 1;
            }
        } else if (arg == "--seed") {
            seed = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            usage(argv[0]); return 1;
        }
    }

    std::cout << "Parsing graph... " << std::flush;
    Graph g = parse_gz(dist_file, time_file, coord_file);
    std::cout << g.num_nodes << " nodes, " << g.num_edges << " edges\n";

    std::cout << "Building " << num_lm << " landmarks"
              << " (metric=" << metric << ", policy=";
    switch (policy) {
        case LandmarkPolicy::Random:            std::cout << "random";    break;
        case LandmarkPolicy::Farthest:          std::cout << "farthest";  break;
        case LandmarkPolicy::OptimizedFarthest: std::cout << "optimized"; break;
        case LandmarkPolicy::Avoid:             std::cout << "avoid";     break;
        case LandmarkPolicy::MaxCover:          std::cout << "maxcover";  break;
    }
    std::cout << ", seed=" << seed << ")... " << std::flush;

    ReverseGraph rg(g);
    LandmarkTable lm = build_landmarks(g, rg, num_lm, metric, policy, seed);

    std::cout << "done\nSaving to " << out_path << "... " << std::flush;
    save_landmarks(lm, out_path);
    std::cout << "done\n";
}
