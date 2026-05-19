#include "bi_obj_a_star.hpp"
#include "graph_parse.hpp"
#include "landmarks.hpp"
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>


static double delta(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

static const char *policy_name(LandmarkPolicy p) {
    switch (p) {
        case LandmarkPolicy::Random: 
            return "random";
        case LandmarkPolicy::Farthest:
            return "farthest";
        case LandmarkPolicy::OptimizedFarthest:
            return "optimized";
        case LandmarkPolicy::Avoid:
            return "avoid";
    }
}

static void usage(const char *prog) {
    std::cerr << "Usage: " << prog
              << " <dist.gr.gz> <time.gr.gz> <coord.co.gz>"
                 " [--queries N] [--lm K] [--seed S]\n"
              << "  --queries N  number of random query pairs (default: 100)\n"
              << "  --lm K       number of landmarks (default: 16)\n"
              << "  --seed S     RNG seed (default: 0)\n";
}

int main(int argc, char *argv[]) {
    if (argc < 4) { usage(argv[0]); return 1; }

    uint32_t num_queries = 100;
    uint32_t num_lm      = 16;
    uint32_t seed        = 0;

    for (int i = 4; i < argc; ++i) {
        std::string arg = argv[i];
        if (i + 1 >= argc && (arg == "--queries" || arg == "--lm" || arg == "--seed")) {
            std::cerr << "Missing value for " << arg << "\n";
            return 1;
        }
        if      (arg == "--queries") num_queries = static_cast<uint32_t>(std::stoul(argv[++i]));
        else if (arg == "--lm") num_lm = static_cast<uint32_t>(std::stoul(argv[++i]));
        else if (arg == "--seed") seed = static_cast<uint32_t>(std::stoul(argv[++i]));
        else { std::cerr << "Unknown argument: " << arg << "\n"; return 1; }
    }

    std::cerr << "[graph] parsing... " << std::flush;
    auto t0 = std::chrono::steady_clock::now();
    Graph g = parse_gz(argv[1], argv[2], argv[3]);
    ReverseGraph rg(g);
    double parse_time = delta(t0);
    std::cerr << g.num_nodes << " nodes, " << g.num_edges << " edges ("
              << std::fixed << std::setprecision(1) << parse_time << " ms)\n\n";

    std::mt19937 qgen(seed);
    std::uniform_int_distribution<uint32_t> rnd(0, static_cast<uint32_t>(g.num_nodes) - 1);
    std::vector<std::pair<uint32_t, uint32_t>> qpairs(num_queries);
    for (auto &[s, t] : qpairs) { s = rnd(qgen); t = rnd(qgen); }

    static const LandmarkPolicy policies[] = {
        LandmarkPolicy::Random,
        LandmarkPolicy::Farthest,
        LandmarkPolicy::OptimizedFarthest,
        LandmarkPolicy::Avoid,
    };

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "# nodes=" << g.num_nodes
              << " edges=" << g.num_edges
              << " lm=" << num_lm
              << " queries=" << num_queries
              << " seed=" << seed << "\n";
    std::cout << "# graph_parse_ms=" << parse_time << "\n\n";

    std::cout << "policy,metric,lm_build_ms\n";

    struct QueryResult { const char *policy; double avg_ms, min_ms, max_ms, avg_sols; };
    std::vector<QueryResult> qresults;
    qresults.reserve(std::size(policies));

    for (auto policy : policies) {
        const char *pname = policy_name(policy);
        std::cerr << "  [" << pname << "]\n";

        std::cerr << "    build metric=1... " << std::flush;
        auto tb1 = std::chrono::steady_clock::now();
        LandmarkTable lm1 = build_landmarks(g, rg, num_lm, 1, policy, seed);
        double b1 = delta(tb1);
        std::cerr << b1 << " ms\n";
        std::cout << pname << ",1," << b1 << "\n";

        std::cerr << "    build metric=2... " << std::flush;
        auto tb2 = std::chrono::steady_clock::now();
        LandmarkTable lm2 = build_landmarks(g, rg, num_lm, 2, policy, seed);
        double b2 = delta(tb2);
        std::cerr << b2 << " ms\n";
        std::cout << pname << ",2," << b2 << "\n";

        std::cerr << "    query (" << num_queries << " pairs)... " << std::flush;
        std::vector<LandmarkTable> lm_vec = {std::move(lm1), std::move(lm2)};
        BOBALT algo(g, rg, lm_vec);

        double total = 0.0, mn = std::numeric_limits<double>::max(), mx = 0.0, sols = 0.0;
        for (auto [src, tgt] : qpairs) {
            auto tq = std::chrono::steady_clock::now();
            auto s  = algo.query(src, tgt);
            double qms = delta(tq);
            total += qms;
            if (qms < mn) mn = qms;
            if (qms > mx) mx = qms;
            sols += static_cast<double>(s.size());
        }
        double avg = total / num_queries;
        std::cerr << avg << " ms avg\n\n";
        qresults.push_back({pname, avg, mn, mx, sols / num_queries});
    }

    std::cout << "\npolicy,avg_query_ms,min_query_ms,max_query_ms,avg_solutions\n";
    for (auto &r : qresults)
        std::cout << r.policy << "," << r.avg_ms << ","
                  << r.min_ms << "," << r.max_ms << ","
                  << r.avg_sols << "\n";

    return 0;
}
