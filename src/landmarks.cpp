#include "graph_parse.hpp"
#include "landmarks.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <string>
#include <vector>
#include <future>
#include <zlib.h>

inline int get_bucket(uint32_t val, uint32_t last) {
    if (val == last) return 0;
    uint32_t diff = val ^ last;
    return 32 - __builtin_clz(diff);
}

uint32_t LandmarkTable::lower_bound(uint32_t u, uint32_t v) const {
    uint32_t lb = 0;
    for (uint32_t l = 0; l < num_landmarks; ++l) {
        if (from(l, v) > from(l, u)) lb = std::max(lb, from(l, v) - from(l, u));
        if (to(l, u)   > to(l, v))   lb = std::max(lb, to(l, u)   - to(l, v));
    }
    return lb;
}

template <typename GraphType>
void dijkstra(const GraphType &g, uint32_t source, int metric,
              uint32_t *dists, uint32_t *parents) {
    std::fill_n(dists, g.num_nodes, INF);
    if (parents) std::fill_n(parents, g.num_nodes, UINT32_MAX);
    dists[source] = 0;

    std::vector<std::pair<uint32_t, uint32_t>> buckets[33];
    uint32_t bucket_min[33];
    std::fill_n(bucket_min, 33, INF);

    uint32_t prev = 0;
    int size = 0;

    buckets[0].push_back({0, source});
    size++;

    while (size > 0) {
        if (buckets[0].empty()) {
            int i = 1;
            while (buckets[i].empty()) i++;
            prev = bucket_min[i];
            for (const auto &p : buckets[i]) {
                int new_idx = get_bucket(p.first, prev);
                buckets[new_idx].push_back(p);
                bucket_min[new_idx] = std::min(bucket_min[new_idx], p.first);
            }
            buckets[i].clear();
            bucket_min[i] = INF;
        }

        auto [d, u] = buckets[0].back();
        buckets[0].pop_back();
        size--;

        if (d > dists[u]) continue;

        for (uint32_t i = edge_begin(g, u); i < edge_end(g, u); ++i) {
            uint32_t v = edge_target(g, i);
            uint32_t weight = edge_weight(g, i, metric);
            if (dists[u] + weight < dists[v]) {
                dists[v] = dists[u] + weight;
                if (parents) parents[v] = u;
                int idx = get_bucket(dists[v], prev);
                buckets[idx].push_back({dists[v], v});
                bucket_min[idx] = std::min(bucket_min[idx], dists[v]);
                size++;
            }
        }
    }
}

uint32_t next_landmark_random(std::mt19937 &gen, uint32_t num_nodes) {
    return std::uniform_int_distribution<uint32_t>(0, num_nodes - 1)(gen);
}

uint32_t next_landmark_farthest(const LandmarkTable &lm) {
    uint32_t best = 0;
    uint32_t best_val = 0;
    for (uint32_t v = 0; v < lm.num_nodes; ++v) {
        uint32_t min_dist = INF;
        for (uint32_t l = 0; l < lm.num_landmarks; ++l)
            min_dist = std::min(min_dist, std::min(lm.to(l, v), lm.from(l, v)));
        if (min_dist > best_val) { best_val = min_dist; best = v; }
    }
    return best;
}

template <typename GraphType>
uint32_t next_landmark_avoid(const GraphType &g, const LandmarkTable &lm,
                             int metric, std::mt19937 &gen,
                             std::vector<uint32_t> &dists,
                             std::vector<uint32_t> &parents) {
    const uint32_t n = g.num_nodes;

    uint32_t root;
    if (lm.num_landmarks == 0) {
        root = next_landmark_random(gen, n);
    } else {
        std::vector<uint32_t> distance(n, 0);
        for (uint32_t v = 0; v < n; ++v) {
            uint32_t min_dist = INF;
            for (uint32_t l = 0; l < lm.num_landmarks; ++l)
                min_dist = std::min(min_dist, std::min(lm.to(l, v), lm.from(l, v)));
            distance[v] = (min_dist < INF) ? min_dist : 0;
        }
        std::discrete_distribution<uint32_t> dist(distance.begin(), distance.end());
        root = dist(gen);
    }

    dijkstra(g, root, metric, dists.data(), parents.data());

    std::vector<std::vector<uint32_t>> children(n);
    std::vector<uint32_t> weight(n, 0);
    for (uint32_t v = 0; v < n; ++v) {
        if (parents[v] != UINT32_MAX && parents[v] != v)
            children[parents[v]].push_back(v);
        if (dists[v] < INF)
            weight[v] = dists[v] - lm.lower_bound(root, v);
    }

    std::vector<int64_t> size(n, 0);
    std::vector<bool> has_lm(n, false);

    for (uint32_t l = 0; l < lm.num_landmarks; ++l)
        has_lm[lm.nodes[l]] = true;

    // Post-order traversal
    std::vector<uint32_t> order;
    order.reserve(n);
    order.push_back(root);
    for (uint32_t i = 0; i < order.size(); ++i)
        for (uint32_t c : children[order[i]])
            order.push_back(c);

    // Compute subtree sizes
    for (uint32_t i = order.size(); i > 0; --i ) {
        uint32_t v = order[i-1];
        if (has_lm[v]) { size[v] = 0; continue; }
        size[v] = weight[v];
        for (uint32_t c : children[v]) {
            has_lm[v] = has_lm[v] || has_lm[c];
            size[v] += size[c];
        }
        if (has_lm[v]) size[v] = 0;
    }

    uint32_t w = root;
    int64_t best_size = -1;
    for (uint32_t v : order)
        if (size[v] > best_size) { best_size = size[v]; w = v; }

    uint32_t cur = w;
    while (!children[cur].empty()) {
        uint32_t next = *std::max_element(
            children[cur].begin(), children[cur].end(),
            [&](uint32_t a, uint32_t b) { return size[a] < size[b]; });
        cur = next;
    }
    return cur;
}

uint32_t farthest_score(const LandmarkTable &lm) {
    uint32_t score = 0;
    for (uint32_t v = 0; v < lm.num_nodes; ++v) {
        uint32_t min_dist = INF;
        for (uint32_t l = 0; l < lm.num_landmarks; ++l)
            min_dist = std::min(min_dist, std::min(lm.to(l, v), lm.from(l, v)));
        if (min_dist < INF) score = std::max(score, min_dist);
    }
    return score;
}


LandmarkTable build_landmarks(const Graph &g, const ReverseGraph &rg,
                              uint32_t num_lm, int metric,
                              LandmarkPolicy policy,
                              uint32_t seed) {
    LandmarkTable lm;
    lm.num_nodes = g.num_nodes;
    lm.nodes.resize(num_lm);
    lm.dist_from.resize(num_lm * g.num_nodes);
    lm.dist_to.resize(num_lm * g.num_nodes);

    std::mt19937 gen(seed);
    std::vector<uint32_t> buf(g.num_nodes);
    std::vector<uint32_t> parents(g.num_nodes);

    if (policy == LandmarkPolicy::Random || policy == LandmarkPolicy::Farthest ||
        policy == LandmarkPolicy::Avoid) {

        for (uint32_t i = 0; i < num_lm; ++i) {
            uint32_t node;
            if (i == 0 || policy == LandmarkPolicy::Random)
                node = next_landmark_random(gen, g.num_nodes);
            else if (policy == LandmarkPolicy::Farthest)
                node = next_landmark_farthest(lm);
            else
                node = next_landmark_avoid(g, lm, metric, gen, buf, parents);

            lm.nodes[i] = node;

            uint32_t *from_ptr = lm.dist_from.data() + i * g.num_nodes;
            uint32_t *to_ptr = lm.dist_to.data() + i * g.num_nodes;
            auto fwd = std::async(std::launch::async,
                [&g, node, metric, from_ptr] { dijkstra(g, node, metric, from_ptr); });
            dijkstra(rg, node, metric, to_ptr);
            fwd.wait();

            lm.num_landmarks = i + 1;
        }
        return lm;
    }

    if (policy == LandmarkPolicy::OptimizedFarthest) {
        for (uint32_t i = 0; i < num_lm; ++i) {
            uint32_t node = (i == 0) ? next_landmark_random(gen, g.num_nodes)
                                     : next_landmark_farthest(lm);
            lm.nodes[i] = node;
            uint32_t *from_ptr = lm.dist_from.data() + i * g.num_nodes;
            uint32_t *to_ptr   = lm.dist_to.data()   + i * g.num_nodes;
            auto fwd = std::async(std::launch::async,
                [&g, node, metric, from_ptr] { dijkstra(g, node, metric, from_ptr); });
            dijkstra(rg, node, metric, to_ptr);
            fwd.wait();
            lm.num_landmarks = i + 1;
        }

        bool any_improved = true;
        while (any_improved) {
            any_improved = false;
            for (uint32_t slot = 0; slot < num_lm; ++slot) {
                const uint32_t n = g.num_nodes;
                std::vector<uint32_t> saved_from(lm.dist_from.begin() + slot * n,
                                                lm.dist_from.begin() + (slot + 1) * n);
                std::vector<uint32_t> saved_to(lm.dist_to.begin() + slot * n,
                                              lm.dist_to.begin() + (slot + 1) * n);

                std::fill(lm.dist_from.begin() + slot * n,
                          lm.dist_from.begin() + (slot + 1) * n, INF);
                std::fill(lm.dist_to.begin() + slot * n,
                          lm.dist_to.begin() + (slot + 1) * n, INF);

                uint32_t prev_score = farthest_score(lm);
                uint32_t candidate = next_landmark_farthest(lm);

                uint32_t *from_ptr = lm.dist_from.data() + slot * n;
                uint32_t *to_ptr = lm.dist_to.data() + slot * n;
                auto fwd = std::async(std::launch::async,
                    [&g, candidate, metric, from_ptr] { dijkstra(g, candidate, metric, from_ptr); });
                dijkstra(rg, candidate, metric, to_ptr);
                fwd.wait();

                if (farthest_score(lm) > prev_score) {
                    lm.nodes[slot] = candidate;
                    any_improved = true;
                } else {
                    std::copy(saved_from.begin(), saved_from.end(),
                              lm.dist_from.begin() + slot * n);
                    std::copy(saved_to.begin(), saved_to.end(),
                              lm.dist_to.begin() + slot * n);
                }
            }
        }
        return lm;
    }
}

static void gz_write_all(gzFile f, const void *buf, size_t len) {
    const char *p = static_cast<const char *>(buf);
    while (len > 0) {
        unsigned chunk = static_cast<unsigned>(std::min(len, size_t(1u << 30)));
        gzwrite(f, p, chunk);
        p += chunk;
        len -= chunk;
    }
}

static void gz_read_all(gzFile f, void *buf, size_t len) {
    char *p = static_cast<char *>(buf);
    while (len > 0) {
        unsigned chunk = static_cast<unsigned>(std::min(len, size_t(1u << 30)));
        gzread(f, p, chunk);
        p += chunk;
        len -= chunk;
    }
}

void save_landmarks(const LandmarkTable &t, const std::string &path) {
    gzFile out = gzopen(path.c_str(), "wb1");
    gz_write_all(out, &t.num_landmarks, sizeof(t.num_landmarks));
    gz_write_all(out, &t.num_nodes, sizeof(t.num_nodes));
    gz_write_all(out, t.nodes.data(), t.nodes.size() * sizeof(uint32_t));
    gz_write_all(out, t.dist_from.data(), t.dist_from.size() * sizeof(uint32_t));
    gz_write_all(out, t.dist_to.data(), t.dist_to.size() * sizeof(uint32_t));
    gzclose(out);
}

LandmarkTable load_landmarks(const std::string &path) {
    gzFile in = gzopen(path.c_str(), "rb");
    LandmarkTable t;
    gz_read_all(in, &t.num_landmarks, sizeof(t.num_landmarks));
    gz_read_all(in, &t.num_nodes, sizeof(t.num_nodes));
    t.nodes.resize(t.num_landmarks);
    t.dist_from.resize(t.num_landmarks * t.num_nodes);
    t.dist_to.resize(t.num_landmarks * t.num_nodes);
    gz_read_all(in, t.nodes.data(), t.nodes.size() * sizeof(uint32_t));
    gz_read_all(in, t.dist_from.data(), t.dist_from.size() * sizeof(uint32_t));
    gz_read_all(in, t.dist_to.data(), t.dist_to.size() * sizeof(uint32_t));
    gzclose(in);
    return t;
}