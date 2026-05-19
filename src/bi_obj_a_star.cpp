#include "bi_obj_a_star.hpp"
#include "graph_parse.hpp"
#include "landmarks.hpp"
#include <algorithm>
#include <future>
#include <limits>
#include <queue>

bool Node::operator>(const Node& other) const {
    if (f1 != other.f1) return f1 > other.f1;
    return f2 > other.f2;
}

BOBALT::BOBALT(const Graph &g, const ReverseGraph &rg, const std::vector<LandmarkTable> &lm) : graph(g), rev_graph(rg), landmarks(lm) {}

uint32_t BOBALT::alt_h(int metric, uint32_t u, uint32_t v) const {
    return landmarks[metric - 1].lower_bound(u, v);
}


template<typename G, typename H1, typename H2>
static Result boa(const G &g, uint32_t source, uint32_t target, bool inverted,
                  H1 h1, H2 h2)
{
    const uint32_t INF = std::numeric_limits<uint32_t>::max();
    const size_t N = g.num_nodes;

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::vector<uint32_t> g1_min(N, INF);
    std::vector<uint32_t> g2_min(N, INF);

    Result result;
    result.lower_bound.assign(N, -1);

    Node start;
    start.state = source;
    start.g1 = 0;
    start.g2 = 0;
    start.f1 = h1(source);
    start.f2 = h2(source);
    open.push(start);

    while (!open.empty()) {
        Node x = open.top();
        open.pop();
        uint32_t u = x.state;

        if (x.f1 >= g1_min[source])
            break;

        if (x.g2 >= g2_min[u] || x.f2 >= g2_min[target]) continue;

        if (result.lower_bound[u] == (uint32_t)-1)
            result.lower_bound[u] = x.g1;

        g2_min[u] = x.g2;

        if (u == target) {
            if (!result.solutions.empty() && result.solutions.back().f1 == x.f1)
                result.solutions.pop_back();
            result.solutions.push_back(x);
            g1_min[source] = x.f1;
            continue;
        }

        for (uint32_t e = g.offset[u]; e < g.offset[u + 1]; ++e) {
            uint32_t v = g.target[e];
            uint32_t w1 = inverted ? g.distance[e] : g.travel_time[e];
            uint32_t w2 = inverted ? g.travel_time[e] : g.distance[e];
            Node y;
            y.state = v;
            y.g1 = x.g1 + w1;
            y.g2 = x.g2 + w2;
            y.f1 = y.g1 + h1(v);
            y.f2 = y.g2 + h2(v);
            if (y.g2 >= g2_min[v] || y.f2 >= g2_min[target]) continue;
            if (y.f1 >= g1_min[source]) continue;
            open.push(y);
        }
    }
    return result;
}

std::vector<Node> BOBALT::query(uint32_t source, uint32_t target) {
    auto h  = [&](int m, uint32_t u, uint32_t v) { return alt_h(m, u, v); };
    auto lb = [](const Result &r, uint32_t u) -> uint32_t {
        uint32_t v = r.lower_bound[u];
        return (v != (uint32_t)-1) ? v : 0;
    };

    auto f1 = std::async(std::launch::async, [&]{ return boa(graph, source, target, true,
        [&](uint32_t u){ return h(1, u, target); },
        [&](uint32_t u){ return h(2, u, target); }); });

    auto f2 = std::async(std::launch::async, [&]{ return boa(rev_graph, target, source, false,
        [&](uint32_t u){ return h(2, source, u); },
        [&](uint32_t u){ return h(1, source, u); }); });

    auto f3 = std::async(std::launch::async, [&]{ return boa(graph, source, target, false,
        [&](uint32_t u){ return h(2, u, target); },
        [&](uint32_t u){ return h(1, u, target); }); });

    auto f4 = std::async(std::launch::async, [&]{ return boa(rev_graph, target, source, true,
        [&](uint32_t u){ return h(1, source, u); },
        [&](uint32_t u){ return h(2, source, u); }); });

    auto r1 = f1.get();
    auto r2 = f2.get();
    auto r3 = f3.get();
    auto r4 = f4.get();

    auto f5 = std::async(std::launch::async, [&]{ return boa(graph, source, target, true,
        [&](uint32_t u){ return std::max<uint32_t>(lb(r4, u), h(1, u, target)); },
        [&](uint32_t u){ return std::max<uint32_t>(lb(r2, u), h(2, u, target)); }); });

    auto f6 = std::async(std::launch::async, [&]{ return boa(rev_graph, target, source, false,
        [&](uint32_t u){ return std::max<uint32_t>(lb(r3, u), h(2, source, u)); },
        [&](uint32_t u){ return std::max<uint32_t>(lb(r1, u), h(1, source, u)); }); });

    auto r5 = f5.get();
    auto r6 = f6.get();

    for (auto &n : r6.solutions) std::swap(n.g1, n.g2);

    std::vector<Node> solutions;
    solutions.reserve(r5.solutions.size() + r6.solutions.size());
    solutions.insert(solutions.end(), r5.solutions.begin(), r5.solutions.end());
    solutions.insert(solutions.end(), r6.solutions.begin(), r6.solutions.end());

    std::sort(solutions.begin(), solutions.end(), [](const Node &a, const Node &b) {
        return a.g1 < b.g1 || (a.g1 == b.g1 && a.g2 < b.g2);
    });

    std::vector<Node> result;
    uint32_t min_g2 = std::numeric_limits<uint32_t>::max();
    for (auto &n : solutions) {
        if (n.g2 < min_g2) {
            result.push_back(n);
            min_g2 = n.g2;
        }
    }
    return result;
}
