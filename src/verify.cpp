#include "bi_obj_a_star.hpp"
#include "graph_parse.hpp"
#include "landmarks.hpp"
#include <cstdint>
#include <iostream>
#include <random>
#include <vector>

int main(int argc, char *argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <dist.gr.gz> <time.gr.gz> <coord.co.gz>"
                     " [--queries N] [--lm K] [--seed S]\n";
        return 1;
    }

    uint32_t num_queries = 50;
    uint32_t num_lm      = 16;
    uint32_t seed        = 42;

    for (int i = 4; i < argc; ++i) {
        std::string arg = argv[i];
        if      (arg == "--queries") num_queries = std::stoul(argv[++i]);
        else if (arg == "--lm")      num_lm      = std::stoul(argv[++i]);
        else if (arg == "--seed")    seed        = std::stoul(argv[++i]);
    }

    std::cerr << "Parsing graph... " << std::flush;
    Graph g = parse_gz(argv[1], argv[2], argv[3]);
    ReverseGraph rg(g);
    std::cerr << g.num_nodes << " nodes\n";

    std::mt19937 gen(seed);
    std::uniform_int_distribution<uint32_t> rnd(0, g.num_nodes - 1);
    std::vector<std::pair<uint32_t,uint32_t>> pairs(num_queries);
    for (auto &[s,t] : pairs) { s = rnd(gen); t = rnd(gen); }

    std::cerr << "Building reference (0 landmarks)...\n";
    LandmarkTable empty1, empty2;
    empty1.num_landmarks = 0; empty1.num_nodes = g.num_nodes;
    empty2.num_landmarks = 0; empty2.num_nodes = g.num_nodes;
    std::vector<LandmarkTable> ref_lm = {std::move(empty1), std::move(empty2)};
    BOBALT ref(g, rg, ref_lm);

    std::cerr << "Building ALT (" << num_lm << " landmarks, farthest)...\n";
    auto lm1 = build_landmarks(g, rg, num_lm, 1, LandmarkPolicy::Farthest, seed);
    auto lm2 = build_landmarks(g, rg, num_lm, 2, LandmarkPolicy::Farthest, seed);
    std::vector<LandmarkTable> alt_lm = {std::move(lm1), std::move(lm2)};
    BOBALT alt(g, rg, alt_lm);

    int pass = 0, fail = 0;
    for (auto [s, t] : pairs) {
        auto ref_sols = ref.query(s, t);
        auto alt_sols = alt.query(s, t);

        bool ok = ref_sols.size() == alt_sols.size();
        if (ok) {
            for (size_t i = 0; i < ref_sols.size(); ++i)
                if (ref_sols[i].g1 != alt_sols[i].g1 || ref_sols[i].g2 != alt_sols[i].g2)
                    { ok = false; break; }
        }

        if (ok) {
            ++pass;
        } else {
            ++fail;
            std::cerr << "FAIL s=" << s << " t=" << t
                      << "  ref=" << ref_sols.size() << " sols"
                      << "  alt=" << alt_sols.size() << " sols\n";
            size_t n = std::max(ref_sols.size(), alt_sols.size());
            for (size_t i = 0; i < n; ++i) {
                if (i < ref_sols.size())
                    std::cerr << "  ref[" << i << "] g1=" << ref_sols[i].g1 << " g2=" << ref_sols[i].g2 << "\n";
                if (i < alt_sols.size())
                    std::cerr << "  alt[" << i << "] g1=" << alt_sols[i].g1 << " g2=" << alt_sols[i].g2 << "\n";
            }
        }
    }

    std::cerr << "\nResult: " << pass << "/" << (pass+fail) << " passed";
    if (fail == 0) std::cerr << "  OK\n";
    else           std::cerr << "  FAILED\n";
    return fail > 0 ? 1 : 0;
}
